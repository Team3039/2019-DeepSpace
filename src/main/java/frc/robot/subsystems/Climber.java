/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.DriveRoller;
import frc.util.PS4Gamepad;

public class Climber extends Subsystem {
  public TalonSRX liftA = new TalonSRX(RobotMap.liftA);
  public TalonSRX liftB = new TalonSRX(RobotMap.liftB);
  public TalonSRX roller = new TalonSRX(RobotMap.roller);

  public Solenoid forklift = new Solenoid(RobotMap.forklift);

  public void driveFrontLift(double power) {
    liftA.set(ControlMode.PercentOutput, power);
    liftB.follow(liftA);
  }

  public void extendRearLift() {
    forklift.set(true);
  }

  public void retractRearLift() {
    forklift.set(false);
  }

  public void driveRoller(PS4Gamepad gp) {
    roller.set(ControlMode.PercentOutput, gp.getLeftYAxis());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveRoller());
  }
}
