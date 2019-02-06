/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.GrabHatch;

public class Shooter extends Subsystem {

  public boolean keepHatch = false;

  public TalonSRX leftShooter = new TalonSRX(RobotMap.leftShooter);
  public TalonSRX rightShooter = new TalonSRX(RobotMap.rightShooter);
  public Solenoid hatchLaunch = new Solenoid(RobotMap.hatchLaunch);
  public Solenoid hatchGrip = new Solenoid(RobotMap.hatchGrip);
  public DigitalInput hatchStatus = new DigitalInput(RobotMap.hatchPressure);

  public void collectCargo() {
    leftShooter.set(ControlMode.PercentOutput, -.55);
    rightShooter.set(ControlMode.PercentOutput, -.55);
  }

  public void stopCargo() {
    leftShooter.set(ControlMode.PercentOutput, 0);
    rightShooter.set(ControlMode.PercentOutput, 0);
    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setNeutralMode(NeutralMode.Brake);

  }

  public void shootCargo() {
    leftShooter.set(ControlMode.PercentOutput, .7);
    rightShooter.set(ControlMode.PercentOutput, .7);
  }

  public void gripHatch() {
    hatchGrip.set(true);
  }

  public void releaseHatch() {
    hatchGrip.set(false);
  }

  public void shootHatch() {
    hatchGrip.set(false);
    hatchLaunch.set(true);
  }

  public void retractShootHatch() {
    hatchLaunch.set(false);
  }

  public boolean getHatchStatus() {
    return hatchStatus.get();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new GrabHatch());
  }
}
