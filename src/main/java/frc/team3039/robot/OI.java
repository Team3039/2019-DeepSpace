package frc.team3039.robot;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.team3039.controllers.GameController;
import frc.team3039.controllers.Playstation;
import frc.team3039.controllers.Xbox;
import frc.team3039.robot.commands.ActuateIntake;
import frc.team3039.robot.commands.Aim;
import frc.team3039.robot.commands.CargoFalseTrigger;
import frc.team3039.robot.commands.ChangeElevatorPosition;
import frc.team3039.robot.commands.LiftBack;
import frc.team3039.robot.commands.LiftFront;
import frc.team3039.robot.commands.ShootCargo;
import frc.team3039.robot.commands.Sequences.CollectCargoSequence;
import frc.team3039.robot.commands.Sequences.ShootHatchSequence;

public class OI {
	//Calls the Gamepad Classes: Defines gp and cp for the robot
	private static OI instance;

	private GameController driverPad;
	private GameController operatorPad;
	
	public static OI getInstance() {
		if (instance == null) {
			instance = new OI();
		}
		return instance;
	}

	private OI() {
		// Driver controller
		driverPad = new GameController(RobotMap.driver, new Playstation());
		operatorPad = new GameController(RobotMap.operator, new Xbox());


		//driver Controls		;

		Button trackGoal = driverPad.getButtonA();
		trackGoal.toggleWhenActive(new Aim());

		Button raiseRobotFront = driverPad.getButtonX();
		raiseRobotFront.toggleWhenPressed(new LiftFront());

		Button raiseRobotBack = driverPad.getButtonB();
		raiseRobotBack.toggleWhenPressed(new LiftBack());

		//Operator Controls
		
		Button elevatorLevel1 = operatorPad.getButtonA();
		elevatorLevel1.whileHeld(new ChangeElevatorPosition(Constants.low));

		Button elevatorLevel2 = operatorPad.getButtonX();
		elevatorLevel2.whileHeld(new ChangeElevatorPosition(Constants.mid));

		Button elevatorLevel3 = operatorPad.getButtonY();
		elevatorLevel3.whileHeld(new ChangeElevatorPosition(Constants.high));

		Button elevatorLevelCargo = operatorPad.getButtonB();
		elevatorLevelCargo.whileHeld(new ChangeElevatorPosition(Constants.cargoShip));

		Button scoreHatch = operatorPad.getLeftBumper();
		scoreHatch.whenPressed(new ShootHatchSequence());

		Button cargoOverride = operatorPad.getLeftTrigger();
		cargoOverride.whileHeld(new CargoFalseTrigger());

		Button scoreCargo = operatorPad.getRightBumper();
		scoreCargo.whileHeld(new ShootCargo());

		Button grabCargo = operatorPad.getRightTrigger();
		grabCargo.whenPressed(new CollectCargoSequence());

		Button actuateIntake = operatorPad.getOptionsButton();
		actuateIntake.toggleWhenPressed(new ActuateIntake());

	}
	
	public GameController getGamepad() {
		return driverPad;
	}

	public GameController getCopad() {
		return operatorPad;
	}
}