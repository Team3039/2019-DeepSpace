package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.CollectCargo;
import frc.robot.commands.LowerElevator;
import frc.robot.commands.RaiseElevator;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.Sequences.ShootHatchSequence;
import frc.util.PS4Copad;
import frc.util.PS4Gamepad;

public class OI {
	//Calls the Gamepad Classes: Defines gp and cp for the robot
	private PS4Gamepad driverPad = new PS4Gamepad(RobotMap.driver);
	private PS4Copad operatorPad = new PS4Copad(RobotMap.operator);
	
	//Returns Controller Data for use with certain Methods
	public PS4Gamepad getGamepad() {
		return driverPad;
	}
	
	public PS4Copad getCopad() {
		return operatorPad;
	}

	public OI() {
		//Driver Buttons
		Button driverTriangle = driverPad.getButtonTriangle();
		Button driverSquare = driverPad.getButtonSquare();
		Button driverCircle = driverPad.getButtonCircle();
		Button driverX = driverPad.getButtonX();
		Button driverShare = driverPad.getShareButton();
		Button driverOptions = driverPad.getOptionsButton();
		Button driverPadButton = driverPad.getButtonPad();
		Button driverL1 = driverPad.getL1();
		Button driverL2 = driverPad.getL2();
		Button driverL3 = driverPad.getL3();
		Button driverR1 = driverPad.getR1();
		Button driverR2 = driverPad.getR2();
		Button driverR3 = driverPad.getR3();

		//Operator Buttons
		Button operatorTriangle = operatorPad.getButtonTriangle();
		Button operatorSquare = operatorPad.getButtonSquare();
		Button operatorCircle = operatorPad.getButtonCircle();
		Button operatorX = operatorPad.getButtonX();
		Button operatorShare = operatorPad.getShareButton();
		Button operatorOptions = operatorPad.getOptionsButton();
		Button operatorPadButton = operatorPad.getButtonPad();
		Button operatorL1 = operatorPad.getL1();
		Button operatorL2 = operatorPad.getL2();
		Button operatorL3 = operatorPad.getL3();
		Button operatorR1 = operatorPad.getR1();
		Button operatorR2 = operatorPad.getR2();
		Button operatorR3 = operatorPad.getR3();

		//driver Controls
			driverL1.whileHeld(new LowerElevator(.5));
			driverR1.whileHeld(new RaiseElevator(.85));
			driverX.whileHeld(new ShootCargo());
			driverCircle.whileHeld(new CollectCargo());
			driverSquare.whenPressed(new ShootHatchSequence());
			driverOptions.toggleWhenPressed(new ActuateIntake());			
			
		//Operator Controls
		// operatorX.whenPressed(new SetElevatorPID(Robot.elevator.low));
		// operatorSquare.whenPressed(new SetElevatorPID(Robot.elevator.mid));
		// operatorTriangle.whenPressed(new SetElevatorPID(Robot.elevator.high));
		// operatorCircle.whenPressed(new CollectCargoSequence());

		// operatorR2.whileHeld(new LowerElevator(.7));
		// operatorR1.whileHeld(new RaiseElevator(.95));
		// operatorL1.whileHeld(new ShootCargo());
		// operatorX.whileHeld(new CollectCargo());
		// operatorL2.toggleWhenPressed(new ActuateIntake());
		// operatorSquare.whenPressed(new ShootHatchSequence());
	}

	
}