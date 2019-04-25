package frc.team3039.auto.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3039.utility.lib.trajectory.LazyLoadTrajectory;

public class LazyLoadCommandGroup extends CommandGroup {
    private ArrayList<LazyLoadTrajectory> trajectories = new ArrayList<LazyLoadTrajectory>();
    
    public LazyLoadTrajectory registerTrajectory(LazyLoadTrajectory trajectory) {
        trajectories.add(trajectory);
        return trajectory;
    }

    public void activate() {
        for(LazyLoadTrajectory trajectory : trajectories) {
            trajectory.activate();
        }
    };

}
