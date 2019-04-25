
package frc.team3039.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3039.robot.Robot;
import frc.team3039.robot.RobotMap;


public class Lights extends Subsystem {

  public DigitalOutput switch1 = new DigitalOutput(RobotMap.switch1);
  public DigitalOutput switch2 = new DigitalOutput(RobotMap.switch2);
  public DigitalOutput allianceSwitch = new DigitalOutput(RobotMap.allianceSwitch);

  public void state0() { switch1.set(false); switch2.set(false); }
  public void state1() { switch1.set(true);  switch2.set(false); }
  public void state2() { switch1.set(false); switch2.set(true);  }
  public void state3() { switch1.set(true);  switch2.set(true);  }

  public void runLights()
  {
    if(Robot.shooter.acquiredHatch)
      state1();
    else if(Robot.shooter.acquiredCargo)
      state2();
    else if(Robot.climber.isClimbing)
      state3();
    else 
      state0();
  }

  public void setRedAlliance()
  {
    allianceSwitch.set(false);
  }

  public void setBlueAlliance()
  {
    allianceSwitch.set(true);
  }

  @Override
  public void initDefaultCommand() {
    
    }
}