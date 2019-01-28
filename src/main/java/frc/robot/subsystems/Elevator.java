
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Elevator extends Subsystem {

  
  public TalonSRX elevatorA = new TalonSRX(RobotMap.elevatorMotorA);
  public TalonSRX elevatorB = new TalonSRX(RobotMap.elevatorMotorB);

  public void driveElevator(double power) {
    elevatorA.set(ControlMode.PercentOutput, power);
    elevatorB.set(ControlMode.PercentOutput, power);
  }

  public void setPIDValues() {
    elevatorA.config_kP(0, .005);
    elevatorA.config_kI(0, .005);
    elevatorA.config_kD(0, .0000001);
    elevatorA.config_kF(0, 0);
  }

  public void setElevatorPID(double position) {
    setPIDValues();
    elevatorA.set(ControlMode.Position, position);
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
