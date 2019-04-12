package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotMap {
//HID
  public final static int driver = 0;
  public final static int operator = 1;

//CAN
  public static int frontleftMotor = 4;
  public static int frontrightMotor = 10;
  public static int rearleftMotor = 5; 
  public static int rearrightMotor = 11; 

  public static int intake = 7;
  public static int leftShooter = 6;
  public static int rightShooter = 0;

  public static int elevatorMotorA = 3;
  public static int elevatorMotorB = 2;
  public static int elevatorMotorC = 1;

//SOLENOID
  public static int hatchGrip = 0;
  public static int hatchLaunch = 1;
  public static int intakeExtension = 2;
  public static int backLift = 3;
  public static int frontLift = 4;
  public static int cameraPivot = 6;

//PWM

//DIO	
  public static int hatchSwitch = 0;
  public static int elevatorLowerLimit = 1;
  public static int elevatorUpperLimit = 2;
  public static int cargoSwitch = 3;

  public static int switch1 = 7;
  public static int switch2 = 8;
  public static int allianceSwitch = 9;
//AIO

//Motor Types
  public static final MotorType driveMotorType = MotorType.kBrushless;
  public static final MotorType elevatorMotorType = MotorType.kBrushless;
}
