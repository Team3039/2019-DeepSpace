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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Intake extends Subsystem {

  public TalonSRX intake = new TalonSRX(RobotMap.intake);
  public Solenoid extension = new Solenoid(RobotMap.intakeExtension);

  public void collectCargo() {
    intake.set(ControlMode.PercentOutput,.6);
  }

  public void stopCargo() {
    intake.setNeutralMode(NeutralMode.Brake);
    intake.set(ControlMode.PercentOutput, 0);
  }

  public void expelCargo() {
    intake.set(ControlMode.PercentOutput, -.6);
  }

  public void extend() {
    extension.set(true);
  }

  public void retract() {
    extension.set(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
