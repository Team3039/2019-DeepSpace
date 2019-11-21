/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot.commands.Sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3039.robot.Constants;
import frc.team3039.robot.commands.ChangeElevatorPosition;
import frc.team3039.robot.commands.RetractIntake;
import frc.team3039.robot.commands.ShootCargo;

public class CargoFaultSequence extends CommandGroup {

  public CargoFaultSequence() {
    addSequential(new ChangeElevatorPosition(Constants.low),1);
    addParallel(new RetractIntake(),.1);
    addSequential(new ShootCargo(),2);
  }
}
