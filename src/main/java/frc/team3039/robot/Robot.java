package frc.team3039.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.robot.loops.Looper;
import frc.team3039.robot.paths.TrajectoryGenerator;
import frc.team3039.robot.subsystems.Climber;
import frc.team3039.robot.subsystems.Drive;
import frc.team3039.robot.subsystems.Drive.DriveControlMode;
import frc.team3039.robot.subsystems.Elevator;
import frc.team3039.robot.subsystems.Intake;
import frc.team3039.robot.subsystems.Lights;
import frc.team3039.robot.subsystems.RobotStateEstimator;
import frc.team3039.robot.subsystems.Shooter;
import frc.team3039.utility.lib.control.RobotStatus;

public class Robot extends TimedRobot {

  //Subsytems
  public static final Drive drive = Drive.getInstance();
  public static final TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();
  public static final RobotStateEstimator estimator = RobotStateEstimator.getInstance();
  public static Elevator elevator = new Elevator();
  public static Lights lights = new Lights();
  public static Shooter shooter = new Shooter();
  public static Intake intake = new Intake();
  public static Climber climber = new Climber();
  public static OI oi;

  //Choosers
  Command autoCommand;
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  private SendableChooser<OperationMode> operationModeChooser;

  // Control looper
  public static final Looper controlLoop = new Looper();

  public static enum OperationMode {
    TEST, PRACTICE, COMPETITION
  };

  public static OperationMode operationMode = OperationMode.COMPETITION;

  // State
  private RobotStatus robotState = RobotStatus.getInstance();

  public Robot() {
    super(Constants.kLooperDt * 2);
    // System.out.println("Main loop period = " + getPeriod());
  }

  @Override
  public void robotInit() {
    oi = new OI().getInstance();
    
    controlLoop.register(drive);
    trajectoryGenerator.generateTrajectories();
    RobotStateEstimator.getInstance().registerEnabledLoops(controlLoop);

    SmartDashboard.putData("Auto mode", autoChooser);
    SmartDashboard.putBoolean("Red Alliance", true);

    operationModeChooser = new SendableChooser<OperationMode>();
    operationModeChooser.addOption("Practice", OperationMode.PRACTICE);
    operationModeChooser.setDefaultOption("Competition", OperationMode.COMPETITION);
    operationModeChooser.addOption("Test", OperationMode.TEST);
    SmartDashboard.putData("Operation Mode", operationModeChooser);

    UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
    usbCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 180, 60);
    
    elevator.setupEncoder();
    System.out.println("Only True Led-Gends Will Know");

  }

  @Override
  public void robotPeriodic() {
    lights.runLights();
    if(SmartDashboard.getBoolean("Red Alliance", true)) {
      lights.setRedAlliance();
    }
    else {
        lights.setBlueAlliance();
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autoCommand = autoChooser.getSelected();
    controlLoop.start();

    if (autoCommand != null) {
      autoCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();

      operationMode = operationModeChooser.getSelected();
      drive.setControlMode(DriveControlMode.JOYSTICK);

      controlLoop.start();
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    SmartDashboard.putNumber("Position", Robot.elevator.getPosition());
    SmartDashboard.putNumber("Velocity", Robot.elevator.getVelocity());
    // SmartDashboard.putNumber("Error", Robot.elevator.elevatorA.getClosedLoopError());
    // System.out.println("\n Hatch Pres  sure is " + Robot.shooter.getHatchStatus());
    // System.out.println("\n  Raising " + Robot.elevator.isRaising);
    // System.out.println("\n  Lowering " + Robot.elevator.isLowering);
    // System.out.println("\n   " + Robot.shooter.acquiredCargo);
    // System.out.println("\n " + Robot.elevator.elevatorA.getAppliedOutput());
    // System.out.println("\n" + Robot.elevator.getVelocity());
    System.out.println("\n" + Robot.elevator.getPosition());
  }

  @Override
  public void testPeriodic() {
  }
}