package frc.team3039.robot;

/**
 * Add your docs here.
 */
public class Constants {

    //Drivetrain 
    public static final double y = .8;
    public static final double rot = .35;

    //Proportional Gain: The distance between the actual position and the setpoint
    public static final double kP_Elevator = 0.045;
    public static final double kP_Vision = 0.02;
    
    //Integral Gain: The area between the actual position and the setpoint over time 
    public static final double kI_Elevator = 0.000001;

    //Derivitive Gain: The instantaneous slope at the current position (smooth your action)
    public static final double kD_Elevator = 0.2;

    //FeedForward Gain: The minimum value to be acheived
    public static final double kF_Elevator = 0.0;

    //Cruise Velocity: The (position/time) to be acheived while executing a motion profile
    public static final int kCrusieVelocity = 6000;

    //Acceleration: The derivitive of Velocity (position/time^2) to be used when entering a motion profile
    public static final int kAcceleration = 6000;
    
    //Ramp Rate: Overrides other Closed Loop Control to gradually increase speed (Seconds)
    public static final double kRampSeconds = 0.35;

    //Elevator Levels
    public static final double intake = -1;
    public static final double low = 0; 
    public static final double mid = 25;
    public static final double high = 50;
    public static final double cargoShip = 18;

    //New Constants
    public static final double kLooperDt = 0.01;

    /* ROBOT PHYSICAL CONSTANTS */

    // r^2:0.8806194352164078 r^2:0.767598614180931
    // ks:0.9161956930322136
    // kv:0.16733812536413087
    // ka:0.044153957047831344

    // r^2:0.9698900328076605
    // r^2:0.8177253879747561
    // ks:0.8636096258554712
    // kv:0.1609164308465849
    // ka:0.046632968167755015

    // r^2:0.9915196195980441
    // r^2:0.6603735758687641
    // ks:1.0915688350353723
    // kv:0.1547622821585623
    // ka:0.04117024510627512

    // New battery 0.5 velocity
    // r^2:0.9950393018114683
    // r^2:0.711344547566271
    // ks:0.928112644250295
    // kv:0.14242500692715937
    // ka:0.03305866811140018

    // Wheels
    // 2019 Robot Values
    public static final double kDriveWheelTrackWidthInches = 28.00; // 22.61;
    public static final double kDriveWheelDiameterInches = 3.922; // 3.875
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 0.9; // 0.924; // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 48.0; // kg TODO tune
    public static final double kRobotAngularInertia = 10.0; // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 0.928112644250295; // V
    public static final double kDriveKv = 0.10305; // 0.14242500692715937; // V per rad/s
    public static final double kDriveKa = 0.01; // 0.011505866811140018; // V per rad/s^2

    // Geometry
    // 2019 Robot Values
    public static final double kCenterToFrontBumperDistance = 15.832; // 31.664/2
    public static final double kCenterToRearBumperDistance = 15.832;
    public static final double kCenterToSideBumperDistance = 15.832;

    /* CONTROL LOOP GAINS */

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0; // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0; // inches per second

    public static final double kPathKX = 4.0;// 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    public static final double kDriveVelocityKp = 0.7; // 0.9;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 3.0; // 10.0;
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static double kDriveVelocityRampRate = 0.05; // 0.05; // 0.02
    public static double kDriveNominalOutput = 0.1;// 0.5 / 12.0;
    public static double kDriveMaxSetpoint = 11.0 * 12.0; // 11 fps

    public static final double kDriveSpinMoveKp = 0.05;
    public static final double kDriveSpinMoveKi = 0.0;
    public static final double kDriveSpinMoveKd = 0.0;
    public static final double kDriveSpinMoveKf = 0.14;
    public static final int kDriveSpinMoveIZone = 200;
    public static final double kDriveSpinMoveKa = 0.001;
    public static final double kDriveSpinMovekTurn = 0.02;

    // 2019 Motion Magic
    // PID gains for elevator velocity loop
    // Units: setpoint, error, and output are in native units per 100ms.
    // Elevator encoder is CTRE mag encoder which is 4096 native units per
    // revolution.
    public static final double kElevatorPositionKp = 0.075; // 0.075;
    public static final double kElevatorPositionKi = 0.00;
    public static final double kElevatorPositionKd = 0.00;
    public static final double kElevatorPositionKf = 0.00;

    public static final double kElevatorMotionMagicKp = 0.4;
    public static final double kElevatorMotionMagicKi = 0.002; // 0.0;
    public static final double kElevatorMotionMagicKd = 0.08; // (4+4)/100;
    public static final double kElevatorMotionMagicKf = 0.12; // 0.197; // 0.06;

    public static final double kElevatorFeedforwardNoBall = 0.035; // 0.197;// 33000;
    public static final double kElevatorFeedforwardWithBall = 0.0; // 0.297;// 33000;
    public static final int kElevatorMaxIntegralAccumulator = 500000; // todo: tune me
    public static final int kElevatorIZone = 200;
    public static final int kElevatorDeadband = 0;
    public static final int kElevatorCruiseVelocity = 10475; // 12500; //Max Velocity 10475
    public static final int kElevatorAcceleration = 15000;// 33000; //Max Velocity / Time to reach top .82
    public static final double kElevatorEpsilon = 1.0;// 33000;
    public static final int kElevatorScurveStrength = 4;
    public static final double kElevatorNominalForward = 0.05;
    public static final double kElevatorNominalReverse = -0.05;
    public static final double kElevatorPeakForward = 1.0;
    public static final double kElevatorPeakReverse = -1.0;

    public final static int REMOTE_0 = 0;
    public final static int REMOTE_1 = 1;
    /*
     * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
     * is auxiliary
     */
    public final static int PID_PRIMARY = 0;
    public final static int PID_TURN = 1;

    public static final double kDriveMotionMagicStraightKp = 0.05;// 0.4;
    public static final double kDriveMotionMagicStraightKi = 0.00; // 0.002; // 0.0;
    public static final double kDriveMotionMagicStraightKd = 4.0; // 0.08; // (4+4)/100;
    public static final double kDriveMotionMagicStraightKf = 0.15; // 0.197; // 0.06;
    public static final int kDriveMotionMagicStraightMaxIntegralAccumulator = 500000; // todo: tune me
    public static final int kDriveMotionMagicStraightIZone = 200;
    public static final int kDriveMotionMagicStraightDeadband = 0;

    public static final double kDriveMotionMagicTurnKp = 0.1;// 0.4;
    public static final double kDriveMotionMagicTurnKi = 0.002; // 0.002; // 0.0;
    public static final double kDriveMotionMagicTurnKd = 0.08; // 0.08; // (4+4)/100;
    public static final double kDriveMotionMagicTurnKf = 0.25; // 0.197; // 0.06;
    public static final int kDriveMotionMagicTurnMaxIntegralAccumulator = 500000; // todo: tune me
    public static final int kDriveMotionMagicTurnIZone = 200;
    public static final int kDriveMotionMagicTurnDeadband = 0;

    public static final int kDriveCruiseVelocity = 5000; // Max Velocity 10475
    public static final int kDriveAcceleration = 6500;// 33000; //Max Velocity / Time to reach top .82

    public static final double kDriveEpsilon = 1.0;// 33000;
    public static final int kDriveScurveStrength = 4;

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; // use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

}
