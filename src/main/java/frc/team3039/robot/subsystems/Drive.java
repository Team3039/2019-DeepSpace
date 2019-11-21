package frc.team3039.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3039.controllers.GameController;
import frc.team3039.robot.Constants;
import frc.team3039.robot.Robot;
import frc.team3039.robot.RobotMap;
import frc.team3039.robot.commands.TeleOpDrive;

public class Drive extends Subsystem {
  private static Drive instance;

    //Drive Motors
    public CANSparkMax frontleftMotor = new CANSparkMax(RobotMap.frontleftMotor, RobotMap.driveMotorType); 
    public CANSparkMax frontrightMotor = new CANSparkMax(RobotMap.frontrightMotor, RobotMap.driveMotorType);
    public CANSparkMax rearleftMotor = new CANSparkMax(RobotMap.rearleftMotor, RobotMap.driveMotorType);
    public CANSparkMax rearrightMotor = new CANSparkMax(RobotMap.rearrightMotor, RobotMap.driveMotorType);

  public void joystickControl(GameController gp) {
    //Tele-Op Driving
    //Each Motor is Set to Brake Mode, the motor speeds are set in an Arcade Drive fashion
    double y = gp.getLeftYAxis()*-Constants.y;
    double rot = gp.getRightXAxis()*Constants.rot;

    //Calculated Outputs (Limits Output to 12V)
    double leftOutput = y + rot;
    double rightOutput = rot - y;


    //Set Motor's Neutral/Idle Mode to Brake
    frontleftMotor.setIdleMode(IdleMode.kBrake);
    frontrightMotor.setIdleMode(IdleMode.kBrake);
    rearrightMotor.setIdleMode(IdleMode.kBrake);
    rearleftMotor.setIdleMode(IdleMode.kBrake);

    //Assigns Each Motor's Power
    frontleftMotor.set(leftOutput);
    frontrightMotor.set(rightOutput);
    rearleftMotor.follow(frontleftMotor);
    rearrightMotor.follow(frontrightMotor);
  }

  public void setTrackingMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //Turns LED on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Begin Processing Vision
  }

  public void setDriverCamMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Turns LED off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); //Disable Vision Processing and Doubles Exposure
  }

  public void aim(GameController gp) {

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

    frontleftMotor.set(y+rot);
    frontrightMotor.set(rot-y);
    rearleftMotor.set(y+rot);
    rearrightMotor.set(rot-y);
  }

  public void stop() {
    //Stops all drive
    frontleftMotor.set(0);
    frontrightMotor.set(0);
    rearleftMotor.set(0);
    rearrightMotor.set(0);   
  }

  public static Drive getInstance() {
    if (instance == null) {
      instance = new Drive();
    }
    return instance;
  }
  
  @Override
  public void initDefaultCommand() {
    // The following command is constantly being run, and thus the gamepad input is constantly being
    // updated as well as the drivetrain speed.
    setDefaultCommand(new TeleOpDrive());
  }
}
