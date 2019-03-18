/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

    //Drivetrain 
    public static final double y = .9;
    public static final double rot = .7;

    //Proportional Gain: The distance between the actual position and the setpoint
    public static final double kP_Elevator = 0.13;
    public static final double kP_Vision = 0.14;
    
    //Integral Gain: The area between the actual position and the setpoint over time 
    public static final double kI_Elevator = 0.000001;

    //Derivitive Gain: The instantaneous slope at the current position (smooth your action)
    public static final double kD_Elevator = 5.0;

    //FeedForward Gain: The minimum value to be acheived
    public static final double kF_Elevator = 0.0;

    //Cruise Velocity: The (position/time) to be acheived while executing a motion profile
    public static final int kCrusieVelocity = 6500;

    //Acceleration: The derivitive of Velocity (position/time^2) to be used when entering a motion profile
    public static final int kAcceleration = 6500;
    
    //Ramp Rate: Overrides other Closed Loop Control to gradually increase speed (Seconds)
    public static final double kRampSeconds = 0.75;

    //Elevator Levels
    public static final double intake = -1500;
    public static final double low = 0; 
    public static final double mid = 17000;
    public static final double high = 31000;
    public static final double cargoShip = 9000;
}
