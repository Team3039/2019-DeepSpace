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

public class Climber extends Subsystem {

  // public Solenoid backLift = new Solenoid(RobotMap.backLift);
  // public Solenoid frontLift = new Solenoid(RobotMap.frontLift);

  public Solenoid suctionPad = new Solenoid(RobotMap.suctionPad);
  public TalonSRX vacuumPump = new TalonSRX(RobotMap.vacuumPump);
  public TalonSRX climberA = new TalonSRX(RobotMap.climberA);
  public TalonSRX climberB = new TalonSRX(RobotMap.climberB);

  public boolean isClimbing = false;

  // public void frontLift(boolean state) {
  //   frontLift.set(state);
  //   // runClimbingLights();
  // }

  // public void rearLift(boolean state) {
  //   backLift.set(state);
  //   // runClimbingLights();
  // }

  // public void runClimbingLights() {
  //   if((!frontLift.get()) && (!backLift.get())) {
  //     isClimbing = false;
  //   }
  //   else {
  //     isClimbing = true;
  //   }
  // }

  public void setSuctionPad(boolean state) {
    suctionPad.set(state);
  }

  public void setVacuumPump(boolean isPumping) {
    if(isPumping) {
      vacuumPump.set(ControlMode.PercentOutput, .8);
    }
    else {
      vacuumPump.set(ControlMode.PercentOutput, 0);
    }
  }

  public void moveLift(double power) {
    climberA.set(ControlMode.PercentOutput, power);
    climberA.setNeutralMode(NeutralMode.Brake);
    climberB.follow(climberA);
  }
  
  @Override
  public void initDefaultCommand() {
  }
}
