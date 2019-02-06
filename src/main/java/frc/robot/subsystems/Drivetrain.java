package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpDrive;
import frc.util.PS4Gamepad;

public class Drivetrain extends Subsystem {


    //Drive Motors
    public TalonSRX frontleftMotor = new TalonSRX(RobotMap.frontleftMotor); 
    public TalonSRX frontrightMotor = new TalonSRX(RobotMap.frontrightMotor);
    public TalonSRX rearleftMotor = new TalonSRX(RobotMap.rearleftMotor);
    public TalonSRX rearrightMotor = new TalonSRX(RobotMap.rearrightMotor);

    //Values
    public boolean isReversed = false;

  public void joystickControl(PS4Gamepad gp) {
    //Tele-Op Driving
    double y = gp.getLeftYAxis()*-.95;
    double rot = gp.getRightXAxis()*.9;

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
    frontleftMotor.set(ControlMode.PercentOutput, 0);
    frontrightMotor.set(ControlMode.PercentOutput, 0);
    rearleftMotor.set(ControlMode.PercentOutput, 0);
    rearrightMotor.set(ControlMode.PercentOutput, 0);   
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleOpDrive());
  }
}
