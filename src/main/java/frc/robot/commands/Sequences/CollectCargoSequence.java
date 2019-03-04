/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ChangeElevatorPosition;
import frc.robot.commands.CollectCargo;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.RetractIntake;

public class CollectCargoSequence extends CommandGroup {

  public CollectCargoSequence() {
    //TODO add timing to SetElevator Commands
    addSequential(new ChangeElevatorPosition(Constants.cargoLow),3);
    addSequential(new ExtendIntake(),5);
    addSequential(new ChangeElevatorPosition(Constants.cargoIntake),3);
    addSequential(new CollectCargo());
    addSequential(new ChangeElevatorPosition(Constants.cargoShip),3);
    addSequential(new RetractIntake(),5);
  }
}
