/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.SetElevatorPID;
import frc.robot.commands.ZeroElevator;

public class ZeroElevatorSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ZeroElevatorSequence() {
    addSequential(new ExtendIntake(),2);
    addSequential(new ZeroElevator());
    addSequential(new SetElevatorPID(Constants.cargoLow));
    addSequential(new RetractIntake());
    addSequential(new SetElevatorPID(Constants.hatchLow));
    }
}
