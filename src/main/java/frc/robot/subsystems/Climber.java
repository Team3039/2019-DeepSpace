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
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Climber extends Subsystem {

  public Talon vacA = new Talon(RobotMap.vacuumA);
  public Talon vacB = new Talon(RobotMap.vacuumB);
  public TalonSRX climberA = new TalonSRX(RobotMap.climberA);
  public TalonSRX climberB = new TalonSRX(RobotMap.climberB);
  public Solenoid armDrop = new Solenoid(RobotMap.suctionArmDrop);
  public DigitalInput liftLimit = new DigitalInput(RobotMap.liftLimit);
  public boolean isClimbing = false;

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

  public void moveLift(double power) {
    climberA.set(ControlMode.PercentOutput, power);
    climberA.setNeutralMode(NeutralMode.Brake);
    climberB.follow(climberA);
  }
  
  public void dropArm() {
    armDrop.set(true);
  }

  public void holdArm() {
    armDrop.set(false);
  }
  
  public boolean getLimit() {
    return !liftLimit.get();
  }

  @Override
  public void initDefaultCommand() {
  }
}
