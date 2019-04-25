package frc.team3039.auto.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3039.utility.lib.control.RobotStatus;

public class WaitUntilCrossXBoundary extends Command {

    public enum MovingXDirection {Negative, Positive};
 
    private double mXBoundary = 0;
    private MovingXDirection mMovingDirection;

    public WaitUntilCrossXBoundary(double x, MovingXDirection movingDirecton) {
        mXBoundary = x;
        mMovingDirection = movingDirecton;
    }

    @Override
    public boolean isFinished() {
        // System.out.println("X Position" +RobotStatus.getInstance().getFieldToVehicle().getTranslation().x() + ", Y Position" +RobotStatus.getInstance().getFieldToVehicle().getTranslation().y());
        if (mMovingDirection == MovingXDirection.Positive) {
            return RobotStatus.getInstance().getFieldToVehicle().getTranslation().x() > mXBoundary;
        }
        else {
            return RobotStatus.getInstance().getFieldToVehicle().getTranslation().x() < mXBoundary;
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        // System.out.println("Passed X Boundary");
    }

    @Override
    public void initialize() {

    }
}
