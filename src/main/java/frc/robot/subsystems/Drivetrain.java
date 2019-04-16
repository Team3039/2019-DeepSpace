package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpDrive;
import frc.util.PS4Gamepad;

public class Drivetrain extends Subsystem {
    //Drive Motors
    public CANSparkMax frontleftDrive = new CANSparkMax(RobotMap.frontleftDrive, RobotMap.driveMotorType); 
    public CANSparkMax frontrightDrive = new CANSparkMax(RobotMap.frontrightDrive, RobotMap.driveMotorType);
    public CANSparkMax rearleftDrive = new CANSparkMax(RobotMap.rearleftDrive, RobotMap.driveMotorType);
    public CANSparkMax rearrightDrive = new CANSparkMax(RobotMap.rearrightDrive, RobotMap.driveMotorType);


  public void joystickControl(PS4Gamepad gp) {
    //Tele-Op Driving
    //Each Motor is Set to Brake Mode, the motor speeds are set in an Arcade Drive fashion
    double y = gp.getLeftYAxis()*-Constants.y;
    double rot = gp.getRightXAxis()*Constants.rot;

    if(Math.abs(y) < .05) {
      y = 0;
    }
    //Calculated Outputs (Limits Output to 12V)
    double leftOutput = y + rot;
    double rightOutput = rot - y;


    //Set Motor's Neutral/Idle Mode to Brake
    frontleftDrive.setIdleMode(IdleMode.kBrake);
    frontrightDrive.setIdleMode(IdleMode.kBrake);
    rearrightDrive.setIdleMode(IdleMode.kBrake);
    rearleftDrive.setIdleMode(IdleMode.kBrake);

    //Assigns Each Motor's Power
    frontleftDrive.set(leftOutput);
    frontrightDrive.set(rightOutput);
    rearleftDrive.follow(frontleftDrive);
    rearrightDrive.follow(frontrightDrive);
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
    double y = -gp.getLeftYAxis()*Constants.y;
    double rot = 0;

    if(Robot.targetValid == 1) {
      rot = kP*errorX; //If LL2 sees a target, use a P Controller to turn to it
    }
    else if(Robot.targetValid == 0) {
      rot = gp.getRightXAxis()*Constants.rot; //If LL2 does not see a target, spin in place
    }

    frontleftDrive.set(y+rot);
    frontrightDrive.set(rot-y);
    rearleftDrive.set(y+rot);
    rearrightDrive.set(rot-y);
  }

  public void stop() {
    //Stops all drive
    frontleftDrive.set(0);
    frontrightDrive.set(0);
    rearleftDrive.set(0);
    rearrightDrive.set(0);   
  }
  
  @Override
  public void initDefaultCommand() {
    // The following command is constantly being run, and thus the gamepad input is constantly being
    // updated as well as the drivetrain speed.
    setDefaultCommand(new TeleOpDrive());
  }
}
