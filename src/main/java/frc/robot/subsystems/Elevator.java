
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends Subsystem {

  
  public TalonSRX elevatorA = new TalonSRX(RobotMap.elevatorMotorA);
  public TalonSRX elevatorB = new TalonSRX(RobotMap.elevatorMotorB);
  public DigitalInput lowerLimit = new DigitalInput(RobotMap.elevatorLowerLimit);
  public DigitalInput upperLimit = new DigitalInput(RobotMap.elevatorUpperLimit);

  public boolean isCargoMode = false; //When true, heights are offset to place Cargo into proper position
  public boolean isRaising = false;
  public boolean isLowering = false;
  public boolean onTarget = false;
  
  //Dynamic Setpoints (Value being changed and read by the CommandGroups)
  public double low = Constants.hatchLow;
  public double mid = Constants.hatchMid;
  public double high = Constants.hatchHigh;
  
  public void driveElevator(double power) {
    elevatorA.setNeutralMode(NeutralMode.Brake);
    elevatorB.setNeutralMode(NeutralMode.Brake);
    elevatorA.set(ControlMode.PercentOutput, power);
    elevatorB.set(ControlMode.PercentOutput, power);
  }

  public void setPIDValues() {
    elevatorA.config_kP(0, Constants.kP_Elevator);
    elevatorA.config_kI(0, Constants.kI_Elevator);
    elevatorA.config_kD(0, Constants.kD_Elevator);
    elevatorA.config_kF(0, Constants.kF_Elevator);
  }

  public void setElevatorPID(double targetPosition) {
    //PID Values
    setPIDValues();

    //Elevator Direction
    if(targetPosition > getPosition()) {
      isRaising = true;
      isLowering = false;
    }
    else if(targetPosition < getPosition()) {
      isRaising = false;
      isLowering = true;
    }
    else {
      isRaising = false;
      isLowering = false;
    }

    //Cargo or Hatch Levels?
    if(isCargoMode) {
      low = Constants.cargoLow;
      mid = Constants.cargoMid;
      high = Constants.cargoHigh;
    }
    else {
      low = Constants.hatchLow;
      mid = Constants.hatchMid;
      high = Constants.hatchHigh;
    }
    
    //Powering Elevator
    if(!getLimit()) {
      elevatorA.set(ControlMode.Position, targetPosition);
      elevatorB.follow(elevatorA);  
    }
    else {
      elevatorA.set(ControlMode.PercentOutput, 0);
      elevatorB.follow(elevatorA);
    }
    
    elevatorA.setNeutralMode(NeutralMode.Brake);
    elevatorB.setNeutralMode(NeutralMode.Brake);
  }

  public void setElevatorMP() {
    setPIDValues();
    elevatorA.set(ControlMode.MotionProfile, 2000);

  }
  public void stopElevator() {
    driveElevator(0);
  }

  public void setupEncoder() {
    elevatorA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public double getPosition() {
    return elevatorA.getSelectedSensorPosition()*-1;
  }

  public boolean getLimit() {
    if(isRaising) {
      return !upperLimit.get();
    }
    if(isLowering) {
      return !lowerLimit.get();
    }
    return false;
  }

  public void resetEncoder() {
    // elevatorA.reset
  }

  @Override
  public void initDefaultCommand() {

  }
}
