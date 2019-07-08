package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotMap {
//HID
  public final static int driver = 0;
  public final static int operator = 1;

//CAN
  public static int frontleftDrive = 4;
  public static int frontrightDrive = 10;
  public static int rearleftDrive = 5; 
  public static int rearrightDrive = 11; 

  public static int intake = 7;
  public static int leftShooter = 6;
  public static int rightShooter = 0;

  public static int elevatorA = 3;
  public static int elevatorB = 2;

  public static int climberA = 8;
  public static int climberB = 9;

//SOLENOID
  public static int hatchGrip = 0;
  public static int hatchLaunch = 1;
  public static int intakeExtension = 2;
  public static int suctionArmDrop = 3;
  public static int cameraPivot = 6;

//PWM
  public static int vacuumA = 0;
  public static int vacuumB = 1;

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
