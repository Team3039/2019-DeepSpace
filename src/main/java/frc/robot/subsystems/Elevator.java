
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.DriveElevator;

public class Elevator extends Subsystem {

  public CANSparkMax elevatorA = new CANSparkMax(RobotMap.elevatorA, RobotMap.elevatorMotorType);
  public CANSparkMax elevatorB = new CANSparkMax(RobotMap.elevatorB, RobotMap.elevatorMotorType);
  // public CANSparkMax elevatorC = new CANSparkMax(RobotMap.elevatorC, RobotMap.elevatorMotorType);

  public CANPIDController pidctrl = elevatorA.getPIDController();
  public CANEncoder encoder = elevatorA.getEncoder();

  public DigitalInput lowerLimit = new DigitalInput(RobotMap.elevatorLowerLimit);
  public DigitalInput upperLimit = new DigitalInput(RobotMap.elevatorUpperLimit);

  public boolean isCargoMode = false; //When true, heights are offset to place Cargo into proper position
  public boolean isRaising = false; //The Elevator is moving up
  public boolean isLowering = false; //The Elevator is coming down
  public boolean isClosedLoopControl = false;

  public double targetPosition = 0;

  //Dynamic Setpoints (Value being changed and read by the Commands, initially set to hatch levels)
  
  public void driveElevatorManual(double power) {
    //Manual drive for the elevator
    elevatorA.setClosedLoopRampRate(0);
    elevatorA.setOpenLoopRampRate(0);

    elevatorA.setIdleMode(IdleMode.kBrake);
    elevatorB.setIdleMode(IdleMode.kBrake);
    // elevatorC.setIdleMode(IdleMode.kBrake);

    elevatorA.set(-power);
    elevatorB.set(-power);  
    // elevatorC.set(-power);
    isClosedLoopControl = false;
  }

  public void driveElevatorClosedLoop() {
    setConstants();

    //Powering Elevator
    elevatorA.setClosedLoopRampRate(Constants.kRampSeconds);
    pidctrl.setReference(-targetPosition, ControlType.kPosition);
    elevatorB.follow(elevatorA); 
    // elevatorC.follow(elevatorA);

    elevatorA.setIdleMode(IdleMode.kBrake);
    elevatorB.setIdleMode(IdleMode.kBrake);
    // elevatorC.setIdleMode(IdleMode.kBrake);

  }

  public void changePosition(double newTarget) {
    isClosedLoopControl = true;
    targetPosition = newTarget;
  }

  public void setConstants() {
    //PID Config 
    pidctrl.setP(Constants.kP_Elevator);
    pidctrl.setI(Constants.kI_Elevator);
    pidctrl.setD(Constants.kD_Elevator);
    pidctrl.setFF(Constants.kF_Elevator);

    if(isLowering) {
      pidctrl.setP(Constants.kP_Elevator/2);
    }
    //Voltage Control
    pidctrl.setOutputRange(-1, 1);
  }

   public void stopElevator() {
    elevatorA.set(0);
    elevatorB.follow(elevatorA);
    // elevatorC.follow(elevatorA);
  }

  public void setupEncoder() {
    // encoder.setPositionConversionFactor(Constants.elevatorPositionConverter);
    // encoder.setVelocityConversionFactor(Constants.elevatorVelocityConverter);
  }

  public double getPosition() {
    return -encoder.getPosition();
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
    encoder.setPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveElevator());
  }
}
