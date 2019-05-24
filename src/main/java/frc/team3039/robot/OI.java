package frc.team3039.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.team3039.robot.commands.ActuateIntake;
import frc.team3039.robot.commands.CargoFalseTrigger;
import frc.team3039.robot.commands.ChangeElevatorPosition;
import frc.team3039.robot.commands.LiftBack;
import frc.team3039.robot.commands.LiftFront;
import frc.team3039.robot.commands.ShootCargo;
import frc.team3039.robot.commands.Sequences.CollectCargoSequence;
import frc.team3039.robot.commands.Sequences.ShootHatchSequence;
import frc.team3039.robot.controller.GameController;
import frc.team3039.robot.controller.Playstation;

public class OI {
	private static OI instance;

	//Calls the Gamepad Classes: Defines gp and cp for the robot
	private GameController driverPad;
	private GameController operatorPad;

	public static OI getInstance() {
		if (instance == null) {
			instance = new OI();
		}
		return instance;
	}

	public OI() {
		driverPad = new GameController(RobotMap.driver, new Playstation());
		operatorPad = new GameController(RobotMap.operator, new Playstation());

		//driver Controls
		Button shootHatch = driverPad.getRightBumper();
		shootHatch.whenPressed(new ShootHatchSequence());

		Button climbFront = driverPad.getButtonSqaure();
		climbFront.toggleWhenPressed(new LiftFront());

		Button climbBack = driverPad.getButtonCircle();
		climbBack.toggleWhenPressed(new LiftBack());

		//Operator Controls
		Button elevatorLowPosition = operatorPad.getButtonX();
		elevatorLowPosition.whileHeld(new ChangeElevatorPosition(Constants.low));

		Button elevstorMidPosition = operatorPad.getButtonSqaure();
		elevstorMidPosition.whileHeld(new ChangeElevatorPosition(Constants.mid));

		Button elevatorHighPosition = operatorPad.getButtonTriangle();
		elevatorHighPosition.whileHeld(new ChangeElevatorPosition(Constants.high));

		Button elevatorCargoPosition = operatorPad.getButtonCircle();
		elevatorCargoPosition.whileHeld(new ChangeElevatorPosition(Constants.cargoShip));

		Button scoreHatch = operatorPad.getLeftBumper();
		scoreHatch.whenPressed(new ShootHatchSequence());

		Button cargoFalseTrigger = operatorPad.getLeftTrigger();
		cargoFalseTrigger.whileHeld(new CargoFalseTrigger());

		Button scoreCargo = operatorPad.getRightBumper();
		scoreCargo.whileHeld(new ShootCargo());

		Button collectCargo = operatorPad.getRightTrigger();
		collectCargo.whenPressed(new CollectCargoSequence());

		Button actuateIntake = operatorPad.getOptionsButton();
		actuateIntake.toggleWhenPressed(new ActuateIntake());

	}

	public GameController getDriverController() {
		return driverPad;
	}

	public GameController getOperatorController() {
		return operatorPad;
	}
}