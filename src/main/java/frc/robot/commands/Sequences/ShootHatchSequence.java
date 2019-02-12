/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.ReleaseHatch;
import frc.robot.commands.ShootHatch;

public class ShootHatchSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ShootHatchSequence() {
    addSequential(new ReleaseHatch(),.1);
    addSequential(new ShootHatch(),.5);
  }
}
