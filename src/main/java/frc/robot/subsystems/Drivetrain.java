package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpDrive;
import frc.util.PS4Gamepad;

public class Drivetrain extends Subsystem {
    //Drive Motors
    public TalonSRX frontleftMotor = new TalonSRX(RobotMap.frontleftMotor); 
    public TalonSRX frontrightMotor = new TalonSRX(RobotMap.frontrightMotor);
    public TalonSRX rearleftMotor = new TalonSRX(RobotMap.rearleftMotor);
    public TalonSRX rearrightMotor = new TalonSRX(RobotMap.rearrightMotor);

    public Solenoid cameraPivot = new Solenoid(RobotMap.cameraPivot);

    //Checks if Robot is being driven in Reverse
    public boolean isReversed = false;

    //Checks if speed needs to be further limited
    public boolean isSlow = false;

  public void joystickControl(PS4Gamepad gp) {
    //Tele-Op Driving
    //Each Motor is Set to Brake Mode, the motor speeds are set in an Arcade Drive fashion
    double y = gp.getLeftYAxis()*-Constants.y;
    double rot = gp.getRightXAxis()*Constants.rot;

    frontleftMotor.setNeutralMode(NeutralMode.Brake);
    frontrightMotor.setNeutralMode(NeutralMode.Brake);
    rearrightMotor.setNeutralMode(NeutralMode.Brake);
    rearleftMotor.setNeutralMode(NeutralMode.Brake);

    frontleftMotor.set(ControlMode.PercentOutput, (y+rot));
    frontrightMotor.set(ControlMode.PercentOutput, (rot-y));
    rearleftMotor.set(ControlMode.PercentOutput, (y+rot));
    rearrightMotor.set(ControlMode.PercentOutput, (rot-y));
    
    if(y < 0) {
      isReversed = true;
    }
    else {
      isReversed = false;
    }
  }


  public void setTrackingMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //Turns LED on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Begin Processing Vision
  }

  public void setDriverCamMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Turns LED off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); //Disable Vision Processing and Doubles Exposure
  }

  public void aim(PS4Gamepad gp) {

    double kP = -Constants.kP_Vision;
    double errorX = -Robot.targetX;
    double y = gp.getLeftYAxis()*-.9;
    double rot = 0;

    if(Robot.targetValid == 1) {
      rot = kP*errorX; //If LL2 sees a target, use a P Controller to turn to it
    }
    else if(Robot.targetValid == 0) {
      rot = gp.getRightXAxis(); //If LL2 does not see a target, spin in place
    }

    frontleftMotor.set(ControlMode.PercentOutput, (y+rot)/2);
    frontrightMotor.set(ControlMode.PercentOutput, (rot-y)/2);
    rearleftMotor.set(ControlMode.PercentOutput, (y+rot)/2);
    rearrightMotor.set(ControlMode.PercentOutput, (rot-y)/2);
  }

  public void stop() {
    //Stops all drive
    frontleftMotor.set(ControlMode.PercentOutput, 0);
    frontrightMotor.set(ControlMode.PercentOutput, 0);
    rearleftMotor.set(ControlMode.PercentOutput, 0);
    rearrightMotor.set(ControlMode.PercentOutput, 0);   
  }

  public void cameraPivot(boolean state) {
    cameraPivot.set(!state);
  }
  
  @Override
  public void initDefaultCommand() {
    // The following command is constantly being run, and thus the gamepad input is constantly being
    // updated as well as the drivetrain speed.
    setDefaultCommand(new TeleOpDrive());
  }
}
