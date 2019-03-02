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
    public static final double y = .95;
    public static final double rot = .9;

    //Elevator 

    //Proportional Gain: The distance between the actual position and the setpoint
    public static final double kP_Elevator = 0.4;
    
    //Integral Gain: The area between the actual position and the setpoint over time 
    public static final double kI_Elevator = 0.0;

    //Derivitive Gain: The instantaneous slope at the current position (smooth your action)
    public static final double kD_Elevator = 0.0;

    //FeedForward Gain: The minimum value to be acheived
    public static final double kF_Elevator = 0.0;

    //Cruise Velocity: The (position/time) to be acheived while executing a motion profile
    public static final int kCrusieVelocity = 9500;

    //Acceleration: The derivitive of Velocity (position/time^2) to be used when entering a motion profile
    public static final int kAcceleration = 15000;
    
    //Elevator Levels
    public static final double cargoIntake = 0;
    public static final double hatchLow = 2000; 
    public static final double hatchMid = 16325;
    public static final double hatchHigh = 33000;
    public static final double cargoLow = 7000;
    public static final double cargoMid = 22000;
    public static final double cargoHigh = 37000;
    public static final double cargoShip = 15000;
}
