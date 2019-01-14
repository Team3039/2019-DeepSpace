package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.LowerElevator;
import frc.robot.commands.RaiseElevator;
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
		Button L1 = driverPad.getL2();
		Button R1 = driverPad.getR2();
		
		//driver Controls
			L1.whileHeld(new LowerElevator());
			R1.whileHeld(new RaiseElevator());
		//Operator Controls
	}

	
}