
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.DriveElevator;

public class Elevator extends Subsystem {

  public TalonSRX elevatorA = new TalonSRX(RobotMap.elevatorMotorA);
  public TalonSRX elevatorB = new TalonSRX(RobotMap.elevatorMotorB);
  public DigitalInput lowerLimit = new DigitalInput(RobotMap.elevatorLowerLimit);
  public DigitalInput upperLimit = new DigitalInput(RobotMap.elevatorUpperLimit);
  public Faults elevatorFaults = new Faults();

  public boolean isCargoMode = false; //When true, heights are offset to place Cargo into proper position
  public boolean isRaising = false; //The Elevator is moving up
  public boolean isLowering = false; //The Elevator is coming down
  public boolean isClosedLoopControl = false;

  public double targetPosition = 0;

  //Dynamic Setpoints (Value being changed and read by the Commands, initially set to hatch levels)
  
  public void driveElevatorManual(double power) {
    //Manual drive for the elevator
    elevatorA.configClosedloopRamp(0);

    elevatorA.setNeutralMode(NeutralMode.Brake);
    elevatorB.setNeutralMode(NeutralMode.Brake);

    elevatorA.set(ControlMode.PercentOutput, power);
    elevatorB.set(ControlMode.PercentOutput, power);  
    isClosedLoopControl = false;
  }

  public void driveElevatorClosedLoop() {
    setConstants();
    
    //Cargo or Hatch Levels?

        
    //Powering Elevator
    elevatorA.configClosedloopRamp(Constants.kRampSeconds);
    elevatorA.set(ControlMode.MotionMagic, targetPosition);
    elevatorB.follow(elevatorA); 

    elevatorA.setNeutralMode(NeutralMode.Brake);
    elevatorB.setNeutralMode(NeutralMode.Brake);
  }

  public void changePosition(double newTarget) {
    isClosedLoopControl = true;
    targetPosition = newTarget;
  }

  public void setConstants() {
    //PID Config 
    elevatorA.config_kP(0, Constants.kP_Elevator);
    elevatorA.config_kP(0, Constants.kP_Elevator);
    elevatorA.config_kI(0, Constants.kI_Elevator);
    elevatorA.config_kD(0, Constants.kD_Elevator);
    elevatorA.config_kF(0, Constants.kF_Elevator);

    //Motion Magic Config
    elevatorA.configMotionCruiseVelocity(Constants.kCrusieVelocity);
    elevatorA.configMotionAcceleration(Constants.kAcceleration);
  }

   public void stopElevator() {
    elevatorA.set(ControlMode.PercentOutput,0);
    elevatorB.follow(elevatorA);
    elevatorA.neutralOutput();
    elevatorB.neutralOutput();
  }

  public void setupEncoder() {
    encoder.setPositionConversionFactor(Constants.elevatorPositionConverter);
    encoder.setVelocityConversionFactor(Constants.elevatorVelocityConverter);
  }

  public double getPosition() {
    return elevatorA.getSelectedSensorPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public boolean getLimit() {
    if(isRaising) {
      return !upperLimit.get(); //If im raising do not hit the top
    }
    if(isLowering) {
      return !lowerLimit.get(); //If im lowering do not bottom out
    }
    return false;
  }

  public void resetEncoder() {
    elevatorA.setSelectedSensorPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveElevator());
  }
}
