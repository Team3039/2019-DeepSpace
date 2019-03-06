/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveElevator extends Command {
  public DriveElevator() {
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.setupEncoder();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(!Robot.elevator.getLimit()) {
      if((Math.abs(Robot.oi.getCopad().getRightYAxis()) > .2)) {
        Robot.elevator.targetPosition = Robot.elevator.getPosition();
        Robot.elevator.isClosedLoopControl = false;
      }
  
      if(Robot.elevator.isClosedLoopControl) {
        Robot.elevator.driveElevatorClosedLoop();
      }
      else {
        Robot.elevator.targetPosition = Robot.elevator.getPosition();
        Robot.elevator.driveElevatorManual(Robot.oi.getCopad().getRightYAxis() * -1);
      }
    }
    else {
      Robot.elevator.stopElevator();
    }

    if(Robot.elevator.isClosedLoopControl) {
      if(Robot.elevator.elevatorA.getClosedLoopError() > 0)  {
        Robot.elevator.isRaising = true;
        Robot.elevator.isLowering = false;
      }
      if(Robot.elevator.elevatorA.getClosedLoopError() < 0)  {
        Robot.elevator.isRaising = false;
        Robot.elevator.isLowering = true;
      }
    }
    if(!Robot.elevator.isClosedLoopControl) {
      if((Robot.oi.getCopad().getRightYAxis()) < -.25)  {
        Robot.elevator.isRaising = true;
        Robot.elevator.isLowering = false;
      }
      if((Robot.oi.getCopad().getRightYAxis()) > .25)  {
        Robot.elevator.isRaising = false;
        Robot.elevator.isLowering = true;
      }
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
