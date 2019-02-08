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
    public static final double rot = .9;

    //Elevator PID Constants
    public static final double kP_Elevator = 0.005;
    public static final double kI_Elevator = 0.005;
    public static final double kD_Elevator = 0.0000001;

    //Elevator Levels
    public static final double cargoIntake = -10;
    public static final double hatchLow = 10; 
    public static final double hatchMid = 20;
    public static final double hatchHigh = 30;
    public static final double cargoLow = 15;
    public static final double cargoMid = 25;
    public static final double cargoHigh = 35;
    public static final double cargoShip = 18;
}
