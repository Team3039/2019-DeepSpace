package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

  public void joystickControl(PS4Gamepad gp) {
    //Tele-Op Driving
    double y = gp.getLeftYAxis()*-.95;
    double x = gp.getRightXAxis()*.8;
    
    frontleftMotor.set(ControlMode.PercentOutput, (y+x)/2);
    frontrightMotor.set(ControlMode.PercentOutput, (x-y)/2);
    rearleftMotor.set(ControlMode.PercentOutput, (y+x)/2);
    rearrightMotor.set(ControlMode.PercentOutput, (x-y)/2); 

    System.out.println("X Value " + x);
    System.out.println("Y Value " +y);
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
