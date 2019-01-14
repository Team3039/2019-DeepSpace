
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Elevator extends Subsystem {

  public TalonSRX elevatorA = new TalonSRX(RobotMap.elevatorMotorA);
  public TalonSRX elevatorB = new TalonSRX(RobotMap.elevatorMotorB);

  public DigitalInput max = new DigitalInput(RobotMap.maxLevel);
  public DigitalInput min = new DigitalInput(RobotMap.minLevel);

  public Encoder encoder = new Encoder(RobotMap.elevatorEncoderA, RobotMap.elevatorEncoderB);

  public void driveElevator(double power) {
    elevatorA.set(ControlMode.PercentOutput, power);
    elevatorB.set(ControlMode.PercentOutput, power);
  }

  public void stopElevator() {
    driveElevator(0);
  }

  public boolean getMax() {
    return !max.get();
  }

  public boolean getMin() {
    return !min.get();
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public double getDistance() {
    return encoder.getDistance()*-1;
  }

  @Override
  public void initDefaultCommand() {

  }
}
