/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3039.robot.RobotMap;

public class Climber extends Subsystem {

  public Solenoid backLift = new Solenoid(RobotMap.backLift);
  public Solenoid frontLift = new Solenoid(RobotMap.frontLift);

  public boolean isClimbing = false;

  public void frontLift(boolean state) {
    frontLift.set(state);
    // runClimbingLights();
  }

  public void rearLift(boolean state) {
    backLift.set(state);
    // runClimbingLights();
  }

  public void runClimbingLights() {
    if((!frontLift.get()) && (!backLift.get())) {
      isClimbing = false;
    }
    else {
      isClimbing = true;
    }
  }

  @Override
  public void initDefaultCommand() {
  }
}
