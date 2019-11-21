/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3039.robot.RobotMap;
import frc.team3039.robot.commands.GrabHatch;

public class Shooter extends Subsystem {
  private static Shooter instance;

  public boolean keepHatch = true;
  public boolean cargoFalseTrigger = false;

  public TalonSRX leftShooter = new TalonSRX(RobotMap.leftShooter);
  public TalonSRX rightShooter = new TalonSRX(RobotMap.rightShooter);
  public Solenoid hatchLaunch = new Solenoid(RobotMap.hatchLaunch);
  public Solenoid hatchGrip = new Solenoid(RobotMap.hatchGrip);
  public DigitalInput hatchStatus = new DigitalInput(RobotMap.hatchSwitch);
  public DigitalInput cargoStatus = new DigitalInput(RobotMap.cargoSwitch);

  //Used for Lighting
  public boolean acquiredCargo = false;
  public boolean acquiredHatch = false;

  public void collectCargo() {
    leftShooter.set(ControlMode.PercentOutput, -.55);
    rightShooter.set(ControlMode.PercentOutput, .55);
    acquiredCargo = false;
  }

  public void stopCargo() {
    leftShooter.set(ControlMode.PercentOutput, -.1);
    rightShooter.set(ControlMode.PercentOutput, .1);
    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setNeutralMode(NeutralMode.Brake);
    acquiredCargo = true;
  }

  public void shootCargo() {
    leftShooter.set(ControlMode.PercentOutput, .65);
    rightShooter.set(ControlMode.PercentOutput, -.65);
    acquiredCargo =false;
  }

  public void setCargoFalseTrigger(boolean state) {
    cargoFalseTrigger = state;
  }

  public void gripHatch() {
    hatchGrip.set(true);
    acquiredHatch = true;
  }

  public void releaseHatch() {
    hatchGrip.set(false);
  }

  public void shootHatch() {
    hatchGrip.set(false);
    hatchLaunch.set(true);
    acquiredHatch = false;
  }

  public void retractShootHatch() {
    hatchLaunch.set(false);
  }

  public boolean getHatchStatus() {
    return !hatchStatus.get();
  }

  public boolean getCargoStatus() {
    return !cargoStatus.get();
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
  
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new GrabHatch());
  }
}
