/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3039.robot.Robot;

public class ZeroElevator extends Command {
  public ZeroElevator() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevator.driveElevatorManual(-.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.elevator.lowerLimit.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.stopElevator();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.elevator.stopElevator();
  }
}
