/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Climber extends Subsystem {

  // public Solenoid backLift = new Solenoid(RobotMap.backLift);
  // public Solenoid frontLift = new Solenoid(RobotMap.frontLift);

  public Solenoid suctionPad = new Solenoid(RobotMap.suctionPad);
  public Talon vacA = new Talon(RobotMap.vacuumA);
  public Talon vacB = new Talon(RobotMap.vacuumB);
  public TalonSRX climberA = new TalonSRX(RobotMap.climberA);
  public TalonSRX climberB = new TalonSRX(RobotMap.climberB);

  public boolean isClimbing = false;

  public void frontLift(boolean state) {
    frontLift.set(state);
  }

  public void rearLift(boolean state) {
    backLift.set(state);
  }

  public void setVacuumPump(boolean isPumping) {
    if(isPumping) {
      vacA.set(.95);
      vacB.set(.95);
    }
    else {
      vacA.set(0);
      vacB.set(0);
    }
  }

  @Override
  public void initDefaultCommand() {
  }
}
