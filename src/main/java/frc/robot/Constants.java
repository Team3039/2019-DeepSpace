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

    //Drivetrain Limiter
    public static final double y = .95;
    public static final double rot = .85;

    //Elevator PID Constants
    public static final double kP_Elevator = 0.7;
    public static final double kI_Elevator = 0.0;
    public static final double kD_Elevator = 0.0;
    public static final double kF_Elevator = 0.0;
    
    //Elevator Levels
    public static final double cargoIntake = 0;
    public static final double hatchLow = 850; 
    public static final double hatchMid = 16850;
    public static final double hatchHigh = 32850;
    public static final double cargoLow = 6850;
    public static final double cargoMid = 21850;
    public static final double cargoHigh = 36850;
    public static final double cargoShip = 14850;
}
