
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Elevator extends Subsystem {

  
  public TalonSRX elevatorA = new TalonSRX(RobotMap.elevatorMotorA);
  public TalonSRX elevatorB = new TalonSRX(RobotMap.elevatorMotorB);

  public boolean isCargoMode = false; //When true, heights are offset to place Cargo into proper position
  public final double hatchLow = 10;
  public final double hatchMid = 20;
  public final double hatchHigh = 30;
  public final double cargoLow = 15;
  public final double cargoMid = 25;
  public final double cargoHigh = 35;
  public final double cargoShip = 18;

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
