package frc.team3039.auto.actions;

import frc.team3039.utility.lib.control.RobotStatus;

public class WaitUntilCrossXBoundaryCommand implements Action {

    private double mXBoundary = 0;

    public WaitUntilCrossXBoundaryCommand(double x) {
        mXBoundary = x;
    }

    @Override
    public boolean isFinished() {
        return RobotStatus.getInstance().getFieldToVehicle().getTranslation().x() > mXBoundary;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {

    }
}
