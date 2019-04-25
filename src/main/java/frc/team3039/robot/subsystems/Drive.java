package frc.team3039.robot.subsystems;

import java.nio.file.Path;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3039.robot.Constants;
import frc.team3039.robot.OI;
import frc.team3039.robot.RobotMap;
import frc.team3039.robot.loops.Loop;
import frc.team3039.robot.planners.DriveMotionPlanner;
import frc.team3039.utility.BHRDifferentialDrive;
import frc.team3039.utility.BHRMathUtils;
import frc.team3039.utility.DriveSignal;
import frc.team3039.utility.MPSoftwarePIDController;
import frc.team3039.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import frc.team3039.utility.MPTalonPIDController;
import frc.team3039.utility.PIDParams;
import frc.team3039.utility.ReflectingCSVWriter;
import frc.team3039.utility.SoftwarePIDController;
import frc.team3039.utility.Util;
import frc.team3039.utility.lib.control.RobotStatus;
import frc.team3039.utility.lib.drivers.TalonSRXEncoder;
import frc.team3039.utility.lib.drivers.TalonSRXFactory;
import frc.team3039.utility.lib.geometry.Pose2d;
import frc.team3039.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3039.utility.lib.geometry.Rotation2d;
import frc.team3039.utility.lib.trajectory.TrajectoryIterator;
import frc.team3039.utility.lib.trajectory.timing.TimedState;

public class Drive extends Subsystem implements Loop {

  private static Drive instance;

  public static enum DriveControlMode {
    JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, VELOCITY_SETPOINT, CAMERA_TRACK, PATH_FOLLOWING, OPEN_LOOP,
  
  };

  // One revolution of the wheel = Pi * D inches = 4096 ticks
  public static final double ENCODER_TICKS_TO_INCHES = 4096.0 / (Constants.kDriveWheelDiameterInches * Math.PI);
  public static final double INCHES_TO_ENCODER_TICKS_MIDDLE_DRIVE = 42.0 / 18.0 * 4096.0 / (2.38 * Math.PI);
  private static final double DRIVE_ENCODER_PPR = 4096.;
  public static final double TRACK_WIDTH_INCHES = 23.92; // 24.56; // 26.937;

  // Motor controllers
  private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();

  // Drive Motors
  public TalonSRXEncoder frontleftMotor;
  public TalonSRXEncoder frontrightMotor;
  public TalonSRX rearleftMotor;
  public TalonSRX rearrightMotor;

  private BHRDifferentialDrive m_drive;

  private boolean mIsBrakeMode = false;

  private long periodMs = (long) (Constants.kLooperDt * 1000.0);

  protected Rotation2d mAngleAdjustment = Rotation2d.identity();

  public static final double STEER_NON_LINEARITY = 0.5;
  public static final double MOVE_NON_LINEARITY = 1.0;

  public static final double STICK_DEADBAND = 0.02;

  public static final double PITCH_THRESHOLD = 20;

  private int pitchWindowSize = 5;
  private int windowIndex = 0;
  private double pitchSum = 0;
  private double[] pitchAverageWindow = new double[pitchWindowSize];

  private int m_moveNonLinear = 0;
  private int m_steerNonLinear = -3;

  private double m_moveScale = 1.0;
  private double m_steerScale = 1.0;

  private double m_moveInput = 0.0;
  private double m_steerInput = 0.0;

  private double m_moveOutput = 0.0;
  private double m_steerOutput = 0.0;

  private double m_moveTrim = 0.0;
  private double m_steerTrim = 0.0;

  public static final double MP_STRAIGHT_T1 = 600;
  public static final double MP_STRAIGHT_T2 = 300;
  public static final double MP_TURN_T1 = 600;
  public static final double MP_TURN_T2 = 300;
  public static final double MP_MAX_TURN_T1 = 200;
  public static final double MP_MAX_TURN_T2 = 100;

  private MPTalonPIDController mpStraightController;
  private PIDParams mpStraightPIDParams = new PIDParams(0.1, 0, 0, 0.005, 0.03, 0.15); // 4 colsons
  // private PIDParams mpStraightPIDParams = new PIDParams(0.05, 0, 0, 0.0008,
  // 0.004, 0.03); // 4 omni
  private PIDParams mpHoldPIDParams = new PIDParams(1, 0, 0, 0.0, 0.0, 0.0);

  private MPSoftwarePIDController mpTurnController; // p i d a v g izone
  // private PIDParams mpTurnPIDParams = new PIDParams(0.035, 0.000, 0.0, 0.00025,
  // 0.00375, 0.0, 100); // 4 colson

  private PIDParams mpTurnPIDParams = new PIDParams(0.005, 0.0001, 0.2, 0.00035, 0.0025, 0.0, 100); // 4 colson
  // wheels

  // private PIDParams mpTurnPIDParams = new PIDParams(0.03, 0.00002, 0.4, 0.0004,
  // 0.0030, 0.0, 100); // 4 omni

  private SoftwarePIDController pidTurnController;
  private PIDParams pidTurnPIDParams = new PIDParams(0.04, 0.001, 0.4, 0, 0, 0.0, 100); // i=0.0008

  private boolean isFinished;
  private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;

  private static final int kPositionControlSlot = 0;
  private static final int kVelocityControlSlot = 1;

  private PigeonIMU gyroPigeon;
  private double[] yprPigeon = new double[3];
  private short[] xyzPigeon = new short[3];
  private boolean useGyroLock;
  private double gyroLockAngleDeg;
  private double kPGyro = 0.04;
  private boolean isCalibrating = false;
  private double gyroOffsetDeg = 0;

  private double mLastValidGyroAngle;
  private double mCameraVelocity = 0;
  private double mCameraSetVelocity = 0;
  private double kCamera = 0.0475; // .7
  private double kCameraDriveClose = 0.08; // .072
  private double kCameraDriveMid = 0.043; // .04
  private double kCameraDriveFar = 0.033; // .04
  private double kCameraClose = 10;
  private double kCameraMid = 15;
  private double kCameraFar = 20;

  // Hardware states //Poofs
  private PeriodicIO mPeriodicIO;
  private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
  private DriveMotionPlanner mMotionPlanner;
  private Rotation2d mGyroOffset = Rotation2d.identity();
  public boolean mOverrideTrajectory = false;
  public double targetDrivePositionTicks;

  // Vision
  public double limeArea;
  public double lastValidLimeArea;
  public double limeX;
  public double limeY;
  public double limeSkew;
  public boolean isLimeValid;
  public double LEDMode;
  public double camMode;
  public boolean onTarget;

  @Override
  public void onStart(double timestamp) {
    synchronized (Drive.this) {
      mpStraightController = new MPTalonPIDController(periodMs, motorControllers);
      mpStraightController.setPID(mpStraightPIDParams, kPositionControlSlot);
      mpTurnController = new MPSoftwarePIDController(periodMs, mpTurnPIDParams, motorControllers);
      pidTurnController = new SoftwarePIDController(pidTurnPIDParams, motorControllers);
    }
  }

  @Override
  public void onStop(double timestamp) {
    // TODO Auto-generated method stub
  }

  @Override
  public void onLoop(double timestamp) {
    synchronized (Drive.this) {
      DriveControlMode currentControlMode = getControlMode();

      if (currentControlMode == DriveControlMode.JOYSTICK) {
        driveWithJoystick();
      } else if (!isFinished()) {
        readPeriodicInputs();
        switch (currentControlMode) {
        case PATH_FOLLOWING:
          updatePathFollower();
          writePeriodicOutputs();
          break;
        case OPEN_LOOP:
          writePeriodicOutputs();
          break;
        case CAMERA_TRACK:
          updateCameraTrack();
          onTarget();
          return;
          case MP_STRAIGHT:
          setFinished(mpStraightController.controlLoopUpdate(getGyroAngleDeg()));
          break;
        case MP_TURN:
          setFinished(mpTurnController.controlLoopUpdate(getGyroAngleDeg()));
          break;
        case PID_TURN:
          setFinished(pidTurnController.controlLoopUpdate(getGyroAngleDeg()));
          break;
        case MANUAL:
          break;
        case VELOCITY_SETPOINT:
          break;
        default:
          System.out.println("Unknown drive control mode: " + currentControlMode);
          break;
        }
      } else {
        // hold in current state
      }
    }
  }

  /**
   * Configures talons for velocity control
   */

  public void configureTalonsForSpeedControl() {
    if (!usesTalonVelocityControl(driveControlMode)) {
      frontleftMotor.enableVoltageCompensation(true);
      frontleftMotor.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
      frontleftMotor.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
      frontleftMotor.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);

      frontrightMotor.enableVoltageCompensation(true);
      frontrightMotor.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
      frontrightMotor.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
      frontrightMotor.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);

      System.out.println("configureTalonsForSpeedControl");
      frontleftMotor.selectProfileSlot(kVelocityControlSlot, TalonSRXEncoder.PID_IDX);
      frontleftMotor.configNominalOutputForward(Constants.kDriveNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
      frontleftMotor.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
      frontleftMotor.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);

      frontrightMotor.selectProfileSlot(kVelocityControlSlot, TalonSRXEncoder.PID_IDX);
      frontrightMotor.configNominalOutputForward(Constants.kDriveNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
      frontrightMotor.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
      frontrightMotor.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
    }
  }

  private void configureMaster(TalonSRX talon) {
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
    talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
    talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
    talon.configNeutralDeadband(0.04, 0);
  }

  private Drive() {
    try {
      mPeriodicIO = new PeriodicIO();

      frontleftMotor = TalonSRXFactory.createTalonEncoder(RobotMap.frontleftMotor, ENCODER_TICKS_TO_INCHES,
          false, FeedbackDevice.CTRE_MagEncoder_Relative);
      rearleftMotor = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.rearleftMotor,
          RobotMap.frontleftMotor);

      frontrightMotor = TalonSRXFactory.createTalonEncoder(RobotMap.frontrightMotor, ENCODER_TICKS_TO_INCHES,
          true, FeedbackDevice.QuadEncoder);
      rearrightMotor = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.rearleftMotor, RobotMap.frontleftMotor);


      frontleftMotor.setSafetyEnabled(false);
      frontleftMotor.setSensorPhase(false);

      frontleftMotor.setInverted(true);
      rearleftMotor.setInverted(true);  //Make sure when the number is positive and you go forward the talon is green

      frontrightMotor.setSafetyEnabled(false);
      frontrightMotor.setSensorPhase(false);

      frontrightMotor.setInverted(false);
      rearrightMotor.setInverted(false);

      configureMaster(frontleftMotor);
      configureMaster(frontrightMotor);

      motorControllers.add(frontleftMotor);
      motorControllers.add(frontrightMotor);

      m_drive = new BHRDifferentialDrive(frontleftMotor, frontrightMotor);
      m_drive.setSafetyEnabled(false);

      mMotionPlanner = new DriveMotionPlanner();

      gyroPigeon = new PigeonIMU(rearrightMotor); //Plug Pigeon into Talon with rear right motor
      gyroPigeon.configFactoryDefault();
      rearrightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

      reloadGains();
      setBrakeMode(true);
    } catch (Exception e) {
      System.err.println("An error occurred in the DriveTrain constructor");
    }
  }


  public static Drive getInstance() {
    if (instance == null) {
      instance = new Drive();
    }
    return instance;
  }

  // Encoder and Gryo Setup
  public static double rotationsToInches(double rotations) {
    return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
  }

  public static double inchesToRotations(double inches) {
    return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
  }

  public double getRightPositionInches() {
    return frontrightMotor.getPositionWorld();
  }

  public double getLeftPositionInches() {
    return frontleftMotor.getPositionWorld();
  }

  private static double radiansPerSecondToTicksPer100ms(double rad_s) {
    return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
  }

  private static double degreesPerSecondToTicksPer100ms(double deg_s) {
    return deg_s / (360.0) * 4096.0 / 10.0;
  }

  private static double inchesPerSecondToTicksPer100ms(double inches_s) {
    return inchesToRotations(inches_s) * 4096.0 / 10.0;
  }

  private static double ticksPer100msToInchesPerSec(double ticks_100ms) {
    return rotationsToInches(ticks_100ms * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLeftEncoderRotations() {
    return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
  }

  public double getRightEncoderRotations() {
    return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
  }

  public double getLeftWheelRotations() {
    return getLeftEncoderRotations();
  }

  public double getRightWheelRotations() {
    return getRightEncoderRotations();
  }

  public double getLeftWheelDistance() {
    return rotationsToInches(getLeftWheelRotations());
  }

  public double getRightWheelDistance() {
    return rotationsToInches(getRightWheelRotations());
  }

  public double getRightVelocityNativeUnits() {
    return mPeriodicIO.right_velocity_ticks_per_100ms;
  }

  public double getRightLinearVelocity() {
    return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLeftVelocityNativeUnits() {
    return mPeriodicIO.left_velocity_ticks_per_100ms;
  }

  public double getLeftLinearVelocity() {
    return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLinearVelocity() {
    return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
  }

  public double getAngularVelocity() {
    return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
  }

  public double getAverageRightLeftVelocity() {
    return (frontleftMotor.getSelectedSensorVelocity() + frontrightMotor.getSelectedSensorVelocity()) / 2;
  }

  public synchronized void resetEncoders() {
    frontrightMotor.setPosition(0);
    frontleftMotor.setPosition(0);
  }

  public void calibrateGyro() {
    gyroPigeon.enterCalibrationMode(CalibrationMode.Temperature, TalonSRXEncoder.TIMEOUT_MS);
  }

  public void endGyroCalibration() {
    if (isCalibrating == true) {
      isCalibrating = false;
    }
  }

  public void setGyroOffset(double offsetDeg) {
    gyroOffsetDeg = offsetDeg;
  }

  public synchronized Rotation2d getGyroAngle() {
    return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(-getGyroAngleDeg()));
  }

  public synchronized void setGyroAngle(Rotation2d adjustment) {
    resetGyro();
    mAngleAdjustment = adjustment;
  }

  public synchronized double getGyroAngleDeg() {
    gyroPigeon.getYawPitchRoll(yprPigeon);
    return -yprPigeon[0] + gyroOffsetDeg;
  }

  public synchronized double getGyroPitchAngle() {
    gyroPigeon.getYawPitchRoll(yprPigeon);
    return yprPigeon[2];
  }

  public short getGyroXAccel() {
    gyroPigeon.getBiasedAccelerometer(xyzPigeon);
    return xyzPigeon[0];
  }

  public short getGyroYAccel() {
    gyroPigeon.getBiasedAccelerometer(xyzPigeon);
    return xyzPigeon[1];
  }

  public short getGyroZAccel() {
    gyroPigeon.getBiasedAccelerometer(xyzPigeon);
    return xyzPigeon[2];
  }

  public boolean checkPitchAngle() {
    double pitchAngle = Math.abs(getGyroPitchAngle());
    if (pitchAngle > 10) {
      return true;
    }
    return false;
  }

  public synchronized void resetGyro() {
    gyroPigeon.setYaw(0, TalonSRXEncoder.TIMEOUT_MS);
    gyroPigeon.setFusedHeading(0, TalonSRXEncoder.TIMEOUT_MS);
  }

  public synchronized Rotation2d getHeading() {
    return mPeriodicIO.gyro_heading;
  }

  public synchronized void setHeading(Rotation2d heading) {
    System.out.println("SET HEADING: " + heading.getDegrees());

    mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(gyroPigeon.getFusedHeading()).inverse());
    System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

    mPeriodicIO.gyro_heading = heading;
  }

  public void zeroSensors() {
    resetEncoders();
    resetGyro();
  }
  // End

  // Auto Setup
  private void setOpenLoopVoltageRamp(double timeTo12VSec) {
    frontleftMotor.configOpenloopRamp(timeTo12VSec, TalonSRXEncoder.TIMEOUT_MS);
    frontrightMotor.configOpenloopRamp(timeTo12VSec, TalonSRXEncoder.TIMEOUT_MS);
  }

  public void setStraightMP(double distanceInches, double maxVelocity, boolean useGyroLock, boolean useAbsolute,
      double desiredAbsoluteAngle) {
    double yawAngle = useAbsolute ? BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), desiredAbsoluteAngle)
        : getGyroAngleDeg();
    mpStraightController.setPID(mpStraightPIDParams, kPositionControlSlot);
    mpStraightController.setPIDSlot(kPositionControlSlot);
    mpStraightController.setMPStraightTarget(0, distanceInches, maxVelocity, MP_STRAIGHT_T1, MP_STRAIGHT_T2,
        useGyroLock, yawAngle, true);
    setControlMode(DriveControlMode.MP_STRAIGHT);
  }

  public void setRelativeTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
    mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec,
        MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
    setControlMode(DriveControlMode.MP_TURN);
  }

  public void setRelativeMaxTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
    mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec,
        MP_MAX_TURN_T1, MP_MAX_TURN_T2, turnType, TRACK_WIDTH_INCHES);
    setControlMode(DriveControlMode.MP_TURN);
  }

  public void setAbsoluteTurnMP(double absoluteTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
    mpTurnController.setMPTurnTarget(getGyroAngleDeg(),
        BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), absoluteTurnAngleDeg), turnRateDegPerSec, MP_TURN_T1,
        MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
    setControlMode(DriveControlMode.MP_TURN);
  }

  public void setAbsoluteMaxTurnMP(double absoluteTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
    mpTurnController.setMPTurnTarget(getGyroAngleDeg(),
        BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), absoluteTurnAngleDeg), turnRateDegPerSec,
        MP_MAX_TURN_T1, MP_MAX_TURN_T2, turnType, TRACK_WIDTH_INCHES);
    setControlMode(DriveControlMode.MP_TURN);
  }

  protected static boolean usesTalonVelocityControl(DriveControlMode state) {
    if (state == DriveControlMode.VELOCITY_SETPOINT || state == DriveControlMode.PATH_FOLLOWING
        || state == DriveControlMode.CAMERA_TRACK) {
      return true;
    }
    return false;
  }

  /**
   * Check if the drive talons are configured for position control
   */
  protected static boolean usesTalonPositionControl(DriveControlMode state) {
    if (state == DriveControlMode.MP_STRAIGHT || state == DriveControlMode.MP_TURN || state == DriveControlMode.HOLD) {
      return true;
    }
    return false;
  }

  public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
    configureTalonsForSpeedControl();
    driveControlMode = DriveControlMode.VELOCITY_SETPOINT;
    updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
  }

  public synchronized void setVelocityNativeUnits(double left_velocity_ticks_per_100ms,
      double right_velocity_ticks_per_100ms) {
    frontleftMotor.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
        mPeriodicIO.left_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0);
    frontrightMotor.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
        mPeriodicIO.right_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0);
  }

  /**
   * Adjust Velocity setpoint (if already in velocity mode)
   * 
   * @param left_inches_per_sec
   * @param right_inches_per_sec
   */
  private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
    if (usesTalonVelocityControl(driveControlMode)) {
      final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
      final double maxSetpoint = Constants.kDriveMaxSetpoint;
      final double scale = max_desired > maxSetpoint ? maxSetpoint / max_desired : 1.0;

      frontleftMotor.setVelocityWorld(left_inches_per_sec * scale);
      frontrightMotor.setVelocityWorld(right_inches_per_sec * scale);

    } else {
      System.out.println("Hit a bad velocity control state");
      frontleftMotor.set(ControlMode.Velocity, 0);
      frontrightMotor.set(ControlMode.Velocity, 0);
    }
  }

  public synchronized void setOpenLoop(DriveSignal signal) {
    if (driveControlMode != DriveControlMode.OPEN_LOOP) {
      setBrakeMode(false);

      System.out.println("Switching to open loop");
      System.out.println(signal);
      driveControlMode = DriveControlMode.OPEN_LOOP;
      frontrightMotor.configNeutralDeadband(0.04, 0);
      frontleftMotor.configNeutralDeadband(0.04, 0);
    }
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = 0.0;
    mPeriodicIO.right_feedforward = 0.0;
  }

  /**
   * Configures talons for velocity control
   */
  public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = feedforward.getLeft();
    mPeriodicIO.right_feedforward = feedforward.getRight();
  }

  public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
    if (mMotionPlanner != null) {
      mOverrideTrajectory = false;
      mMotionPlanner.reset();
      mMotionPlanner.setTrajectory(trajectory);

      // We entered a velocity control state.
      setBrakeMode(true);
      frontleftMotor.selectProfileSlot(kVelocityControlSlot, 0);
      frontrightMotor.selectProfileSlot(kVelocityControlSlot, 0);
      frontleftMotor.configNeutralDeadband(0.0, 0);
      frontrightMotor.configNeutralDeadband(0.0, 0);

      setControlMode(DriveControlMode.PATH_FOLLOWING);
    }
  }

  public boolean isDoneWithTrajectory() {
    if (mMotionPlanner == null) {
      return false;
    }
    return mMotionPlanner.isDone() || mOverrideTrajectory == true;
  }

  public void overrideTrajectory(boolean value) {
    mOverrideTrajectory = value;
  }

  private void updatePathFollower() {
    if (driveControlMode == DriveControlMode.PATH_FOLLOWING) {
      final double now = Timer.getFPGATimestamp();

      DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotStatus.getInstance().getFieldToVehicle());

      // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0,
      // demand.right_feedforward_voltage / 12.0);

      mPeriodicIO.error = mMotionPlanner.error();
      mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

      if (!mOverrideTrajectory) {
        setVelocity(
            new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity),
                radiansPerSecondToTicksPer100ms(output.right_velocity)),
            new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

        mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
        mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
      } else {
        setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
        mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
      }
    } else {
      DriverStation.reportError("Drive is not in path following state", false);
    }
  }

  public synchronized void reloadGains() {
    frontleftMotor.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
    frontleftMotor.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
    frontleftMotor.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
    frontleftMotor.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
    frontleftMotor.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);

    frontrightMotor.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
    frontrightMotor.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
    frontrightMotor.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
    frontrightMotor.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
    frontrightMotor.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);
  }

  public void writeToLog() {
  }

  public synchronized void readPeriodicInputs() {
    double prevLeftTicks = mPeriodicIO.left_position_ticks;
    double prevRightTicks = mPeriodicIO.right_position_ticks;
    mPeriodicIO.left_position_ticks = frontleftMotor.getSelectedSensorPosition(0);
    mPeriodicIO.right_position_ticks = frontrightMotor.getSelectedSensorPosition(0);
    mPeriodicIO.left_velocity_ticks_per_100ms = frontleftMotor.getSelectedSensorVelocity(0);
    mPeriodicIO.right_velocity_ticks_per_100ms = frontrightMotor.getSelectedSensorVelocity(0);
    mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(gyroPigeon.getFusedHeading()).rotateBy(mGyroOffset);

    double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
    if (deltaLeftTicks > 0.0) {
      mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
    } else {
      mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
    }

    double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
    if (deltaRightTicks > 0.0) {
      mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
    } else {
      mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

    }

    if (mCSVWriter != null) {
      mCSVWriter.add(mPeriodicIO);
    }

    // System.out.println("control state: " + mDriveControlState + ", left: " +
    // mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
  }

  public synchronized void writePeriodicOutputs() {
    if (driveControlMode == DriveControlMode.OPEN_LOOP) {
      frontleftMotor.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
      frontrightMotor.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
    } else {
      frontleftMotor.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
          mPeriodicIO.left_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0);
      frontrightMotor.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
          mPeriodicIO.right_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0);
    }
  }
  // End

  // Drive
  public synchronized DriveControlMode getControlMode() {
    return driveControlMode;
  }

  public synchronized void setControlMode(DriveControlMode controlMode) {
    this.driveControlMode = controlMode;
    if (controlMode == DriveControlMode.HOLD) {
      mpStraightController.setPID(mpHoldPIDParams, kPositionControlSlot);
      frontleftMotor.setPosition(0);
      frontleftMotor.set(ControlMode.Position, 0);
      frontrightMotor.setPosition(0);
      frontrightMotor.set(ControlMode.Position, 0);
    }
    setFinished(false);
  }

  public synchronized void setSpeed(double speed) {
    if (speed == 0) {
      setControlMode(DriveControlMode.JOYSTICK);
    } else {
      setControlMode(DriveControlMode.MANUAL);
      frontrightMotor.set(ControlMode.PercentOutput, speed);
      frontleftMotor.set(ControlMode.PercentOutput, speed);
    }
  }

  public void setDriveHold(boolean status) {
    if (status) {
      setControlMode(DriveControlMode.HOLD);
    } else {
      setControlMode(DriveControlMode.JOYSTICK);
    }
  }

  public synchronized void driveWithJoystick() {
    if (m_drive == null)
      return;

    boolean cameraTrackTapeButton = OI.getInstance().driverPad.getR2().get();

    m_moveInput = OI.getInstance().driverPad.getLeftYAxis(); //Slow down the speed just divide the output
    m_steerInput = -OI.getInstance().driverPad.getRightXAxis();

    m_moveOutput = adjustForSensitivity(m_moveScale, m_moveTrim, m_moveInput, m_moveNonLinear, MOVE_NON_LINEARITY);
    m_steerOutput = adjustForSensitivity(m_steerScale, m_steerTrim, m_steerInput, m_steerNonLinear,
        STEER_NON_LINEARITY);

    if (useGyroLock) {
      double yawError = gyroLockAngleDeg - getGyroAngleDeg();
      m_steerOutput = kPGyro * yawError;
    }

    double pitchAngle = updatePitchWindow();
    if (Math.abs(pitchAngle) > PITCH_THRESHOLD) {
      m_moveOutput = Math.signum(pitchAngle) * -1.0;
      m_steerOutput = 0;
      // System.out.println("Pitch Treshhold 2 angle = " + pitchAngle);
    }

    if (cameraTrackTapeButton) {
      setPipeline(2);
      setLimeLED(0);
      updateLimelight();
      double cameraSteer = 0;
      if (isLimeValid) {
        double kCameraDrive = kCameraDriveClose;
        if (limeX <= kCameraClose) {
          kCameraDrive = kCameraDriveClose;
        } else if (limeX < kCameraMid) {
          kCameraDrive = kCameraDriveMid;
        } else if (limeX < kCameraFar) {
          kCameraDrive = kCameraDriveFar;
        }
        cameraSteer = limeX * kCameraDrive;
      } else {
        cameraSteer = -m_steerOutput;
      }
      m_steerOutput = -cameraSteer;
    }

    m_drive.arcadeDrive(-m_moveOutput, -m_steerOutput);
  }

  public boolean isBrakeMode() {
    return mIsBrakeMode;
  }

  public synchronized void setBrakeMode(boolean on) {
    if (mIsBrakeMode != on) {
      mIsBrakeMode = on;
      frontrightMotor.setNeutralMode(NeutralMode.Brake);
      rearrightMotor.setNeutralMode(NeutralMode.Brake);
      frontleftMotor.setNeutralMode(NeutralMode.Brake);
      rearleftMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  private double updatePitchWindow() {
    double lastPitchAngle = pitchAverageWindow[windowIndex];
    double currentPitchAngle = getGyroPitchAngle();
    pitchAverageWindow[windowIndex] = currentPitchAngle;
    pitchSum = pitchSum - lastPitchAngle + currentPitchAngle;

    windowIndex++;
    if (windowIndex == pitchWindowSize) {
      windowIndex = 0;
    }

    return pitchSum / pitchWindowSize;
  }

  private boolean inDeadZone(double input) {
    boolean inDeadZone;
    if (Math.abs(input) < STICK_DEADBAND) {
      inDeadZone = true;
    } else {
      inDeadZone = false;
    }
    return inDeadZone;
  }

  public double adjustForSensitivity(double scale, double trim, double steer, int nonLinearFactor,
      double wheelNonLinearity) {
    if (inDeadZone(steer))
      return 0;

    steer += trim;
    steer *= scale;
    steer = limitValue(steer);

    int iterations = Math.abs(nonLinearFactor);
    for (int i = 0; i < iterations; i++) {
      if (nonLinearFactor > 0) {
        steer = nonlinearStickCalcPositive(steer, wheelNonLinearity);
      } else {
        steer = nonlinearStickCalcNegative(steer, wheelNonLinearity);
      }
    }
    return steer;
  }

  private double limitValue(double value) {
    if (value > 1.0) {
      value = 1.0;
    } else if (value < -1.0) {
      value = -1.0;
    }
    return value;
  }

  private double nonlinearStickCalcPositive(double steer, double steerNonLinearity) {
    return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer) / Math.sin(Math.PI / 2.0 * steerNonLinearity);
  }

  private double nonlinearStickCalcNegative(double steer, double steerNonLinearity) {
    return Math.asin(steerNonLinearity * steer) / Math.asin(steerNonLinearity);
  }

  public synchronized boolean isFinished() {
    return isFinished;
  }

  public synchronized void setFinished(boolean isFinished) {
    this.isFinished = isFinished;
  }
  // End

  // Vision
  public NetworkTable getLimetable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  // Set the LED mode of the limelight
  /*
   * 0- Default setting in pipeline 1- Force Off 2- Force Blink 3- Force On
   */

  public void setLimeLED(int ledMode) {
    getLimetable().getEntry("ledMode").setNumber(ledMode);
  }

  // Set the camera mode
  /*
   * 0- Vision Mode 1- Driver Mode
   */
  public void setLimeCameraMode(int camMode) {
    getLimetable().getEntry("camMode").setNumber(camMode);
  }

  // Set vision pipeline
  // 0-9
  public void setPipeline(int pipeline) {
    getLimetable().getEntry("pipeline").setNumber(pipeline);
  }

  public void setStream(String stream) {
    getLimetable().getEntry("stream").setString(stream);
  }

  // Checks if vison targets are found
  public boolean isValid() {
    return isLimeValid;
  }

  public void updateLimelight() {
    NetworkTable limeTable = getLimetable();
    limeX = limeTable.getEntry("tx").getDouble(0);
    limeY = limeTable.getEntry("ty").getDouble(0);
    limeArea = limeTable.getEntry("ta").getDouble(0);
    limeSkew = limeTable.getEntry("ts").getDouble(0);
    double valid = limeTable.getEntry("tv").getDouble(0);
    if (valid == 0) { // || limeArea > 38
      isLimeValid = false;
    } else if (valid == 1) {
      isLimeValid = true;
      lastValidLimeArea = limeArea;
    }
  }

  /**
   * Called periodically when the robot is in camera track mode.
   */
  private void updateCameraTrack() {
    updateLimelight();
    double deltaVelocity = 0;

    // Ramp velocity down to scaled velocity
    if (mCameraVelocity > mCameraSetVelocity) {
      mCameraVelocity -= 1;
      // System.out.println("Camera velocity = " + mCameraVelocity);
    }

    if (isLimeValid) {
      deltaVelocity = limeX * kCamera * mCameraVelocity;
      mLastValidGyroAngle = getGyroAngleDeg();

      // System.out.println("Valid lime angle = " + limeX);
    } else {
      deltaVelocity = (getGyroAngleDeg() - mLastValidGyroAngle) * kCamera * mCameraVelocity;
      // System.out.println("In Valid lime angle = " + limeX);
    }
    // setVelocityNativeUnits(mCameraVelocity + deltaVelocity, mCameraVelocity -
    // deltaVelocity);
    updateVelocitySetpoint(mCameraVelocity + deltaVelocity, mCameraVelocity - deltaVelocity);
  }

  /**
   * Configures the drivebase to drive a path. Used for autonomous driving
   * 
   * @see Path
   */
  public synchronized void setCameraTrack(double velocityScale) {
    double straightVelocity = (frontleftMotor.getVelocityWorld() + frontrightMotor.getVelocityWorld()) / 2;
    mCameraSetVelocity = velocityScale * straightVelocity;
    setFinished(false);
    driveControlMode = DriveControlMode.CAMERA_TRACK;
    mLastValidGyroAngle = getGyroAngleDeg();
    mCameraVelocity = straightVelocity;
  }

  public synchronized void setCameraTrackWithVelocity(double velocityInchesPerSec) {
    double straightVelocity = inchesPerSecondToTicksPer100ms(velocityInchesPerSec);
    setFinished(false);
    driveControlMode = DriveControlMode.CAMERA_TRACK;
    mLastValidGyroAngle = getGyroAngleDeg();
    mCameraVelocity = straightVelocity;
    mCameraSetVelocity = mCameraVelocity;
  }

  public boolean onTarget() {
    if (limeX < 5) {
      onTarget = true;
    } else {
      onTarget = false;
    }
    return onTarget;
  }

  // End

  public double getPeriodMs() {
    return periodMs;
  }

  @Override
  public void initDefaultCommand() {
  }

  public synchronized void startLogging() {
    if (mCSVWriter == null) {
      mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
    }
  }

  public synchronized void stopLogging() {
    if (mCSVWriter != null) {
      mCSVWriter.flush();
      mCSVWriter = null;
    }
  }

  private int getDriveEncoderTicks(double positionInches) {
    return (int) (positionInches * ENCODER_TICKS_TO_INCHES);
  }

  public synchronized boolean hasFinishedDriveMotionMagic() {
    return Util.epsilonEquals(frontrightMotor.getActiveTrajectoryPosition(), targetDrivePositionTicks, 5);
  }

  public synchronized double getDriveMotionMagicPosition() {
    return frontrightMotor.getActiveTrajectoryPosition();
  }

  public static class PeriodicIO {
    // INPUTS
    public int left_position_ticks;
    public int right_position_ticks;
    public double left_distance;
    public double right_distance;
    public int left_velocity_ticks_per_100ms;
    public int right_velocity_ticks_per_100ms;
    public Rotation2d gyro_heading = Rotation2d.identity();
    public Pose2d error = Pose2d.identity();

    // OUTPUTS
    public double left_demand;
    public double right_demand;
    public double left_accel;
    public double right_accel;
    public double left_feedforward;
    public double right_feedforward;
    public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(
        Pose2dWithCurvature.identity());
  }
  public static void main(String[] args) {

    Drive drive = Drive.getInstance();

    for (int i = 0; i < 10; i++) {
      double m_steerOutput = drive.adjustForSensitivity(1.0, 0, (double) i / 10.0, -3, STEER_NON_LINEARITY);
      System.out.println("i=" + i + ", output=" + m_steerOutput);
    }
  }

}
