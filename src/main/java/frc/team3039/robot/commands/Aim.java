package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3039.robot.Robot;

public class Aim extends Command {
  public Aim() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.setTrackingMode();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.aim(Robot.oi.getGamepad());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.setDriverCamMode();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drive.setDriverCamMode();
  }
}
