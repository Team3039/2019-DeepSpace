/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3039.robot.Robot;

public class ChangeElevatorPosition extends Command {
  double targetPosition;

  public ChangeElevatorPosition(double targetPosition) {
    this.targetPosition = targetPosition;
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
    if(Robot.shooter.acquiredCargo) {
      Robot.elevator.changePosition(targetPosition+9.75);
    }
    else {
      Robot.elevator.changePosition(targetPosition);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return Math.abs(targetPosition - Robot.elevator.getPosition()) < 250;
    return false;
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
