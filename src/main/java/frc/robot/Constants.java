package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

    //Drivetrain 
    public static final double y = .8;
    public static final double rot = .35;

    //Proportional Gain: The distance between the actual position and the setpoint
    public static final double kP_Elevator = 0.085;
    public static final double kP_Vision = 0.02;
    
    //Integral Gain: The area between the actual position and the setpoint over time 
    public static final double kI_Elevator = 0.000001;

    //Derivitive Gain: The instantaneous slope at the current position (smooth your action)
    public static final double kD_Elevator = 5.0;

    //FeedForward Gain: The minimum value to be acheived
    public static final double kF_Elevator = 0.0;

    //Cruise Velocity: The (position/time) to be acheived while executing a motion profile
    public static final int kCrusieVelocity = 9000;

    //Acceleration: The derivitive of Velocity (position/time^2) to be used when entering a motion profile
    public static final int kAcceleration = 11000;
    
    //Ramp Rate: Overrides other Closed Loop Control to gradually increase speed (Seconds)
    public static final double kRampSeconds = 0.3;

    //Elevator Levels
    public static final double intake = -1650;
    public static final double low = 0; 
    public static final double mid = 18000;
    public static final double high = 32500;
    public static final double cargoShip = 10000;

    //Encoder Converters
    public static final double elevatorPositionConverter = 0;
    public static final double elevatorVelocityConverter = 0;

}
