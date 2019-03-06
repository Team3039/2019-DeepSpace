/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;

public class CargoFaultSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CargoFaultSequence() {
    addSequential(new ChangeElevatorPosition(Constants.cargoLow),1);
    addParallel(new RetractIntake(),.1);
    addSequential(new ShootCargo(),2);
  }
}
