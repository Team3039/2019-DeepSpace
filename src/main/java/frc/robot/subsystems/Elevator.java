
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends Subsystem {

  
  public TalonSRX elevatorA = new TalonSRX(RobotMap.elevatorMotorA);
  public TalonSRX elevatorB = new TalonSRX(RobotMap.elevatorMotorB);

  public boolean isCargoMode = false; //When true, heights are offset to place Cargo into proper position
  
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
    elevatorA.config_kF(0, 0);
  }

  public void setElevatorPID(double position) {
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
    
    setPIDValues();
    elevatorA.set(ControlMode.Position, position);
    elevatorA.setNeutralMode(NeutralMode.Brake);
    elevatorB.setNeutralMode(NeutralMode.Brake);
    elevatorB.follow(elevatorA);
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
    return elevatorA.getSelectedSensorPosition();
  }

  public void resetEncoder() {
    // elevatorA.reset
  }

  @Override
  public void initDefaultCommand() {

  }
}
