/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CollectCargo;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.SetElevator;

public class CollectCargoSequence extends CommandGroup {

  public CollectCargoSequence() {
    addSequential(new SetElevator(Constants.cargoLow));
    addSequential(new ExtendIntake(),5);
    addSequential(new SetElevator(Constants.cargoIntake));
    addSequential(new CollectCargo());
    addSequential(new SetElevator(Constants.cargoShip));
    addSequential(new RetractIntake());
  }
}
