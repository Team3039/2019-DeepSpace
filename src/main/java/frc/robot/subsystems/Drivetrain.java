package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpDrive;
import frc.util.PS4Gamepad;

public class Drivetrain extends Subsystem {
    //Drive Motors
    public TalonSRX frontleftMotor = new TalonSRX(RobotMap.frontleftMotor); 
    public TalonSRX frontrightMotor = new TalonSRX(RobotMap.frontrightMotor);
    public TalonSRX rearleftMotor = new TalonSRX(RobotMap.rearleftMotor);
    public TalonSRX rearrightMotor = new TalonSRX(RobotMap.rearrightMotor);

    //Checks if Robot is being driven in Reverse
    public boolean isReversed = false;

  public void joystickControl(PS4Gamepad gp) {
    //Tele-Op Driving
    //Each Motor is Set to Brake Mode, the motor speeds are set in an Arcade Drive fashion
    double y = gp.getLeftYAxis()*-Constants.y;
    double rot = gp.getRightXAxis()*Constants.rot;

    frontleftMotor.setNeutralMode(NeutralMode.Brake);
    frontrightMotor.setNeutralMode(NeutralMode.Brake);
    rearrightMotor.setNeutralMode(NeutralMode.Brake);
    rearleftMotor.setNeutralMode(NeutralMode.Brake);

    frontleftMotor.set(ControlMode.PercentOutput, (y+rot)/2);
    frontrightMotor.set(ControlMode.PercentOutput, (rot-y)/2);
    rearleftMotor.set(ControlMode.PercentOutput, (y+rot)/2);
    rearrightMotor.set(ControlMode.PercentOutput, (rot-y)/2);
    
    if(y < 0) {
      isReversed = true;
    }
    else {
      isReversed = false;
    }
  }

  public void stop() {
    //Stops all drive
    frontleftMotor.set(ControlMode.PercentOutput, 0);
    frontrightMotor.set(ControlMode.PercentOutput, 0);
    rearleftMotor.set(ControlMode.PercentOutput, 0);
    rearrightMotor.set(ControlMode.PercentOutput, 0);   
  }

  @Override
  public void initDefaultCommand() {
    // The following command is constantly being run, and thus the gamepad input is constantly being
    // updated as well as the drivetrain speed.
    setDefaultCommand(new TeleOpDrive());
  }
}
