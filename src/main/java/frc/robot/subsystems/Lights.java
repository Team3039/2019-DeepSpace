
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class Lights extends Subsystem {

  public DigitalOutput switch1 = new DigitalOutput(RobotMap.switch1);
  public DigitalOutput switch2 = new DigitalOutput(RobotMap.switch2);
  public DigitalOutput switch3 = new DigitalOutput(RobotMap.switch3);

  public void state0() { switch1.set(false); switch2.set(false); switch3.set(false); }
  public void state1() { switch1.set(true);  switch2.set(false); switch3.set(false);  }
  public void state2() { switch1.set(false); switch2.set(true);  switch3.set(false); }
  public void state3() { switch1.set(true);  switch2.set(true);  switch3.set(false);  }
  public void state4() { switch1.set(false); switch2.set(false); switch3.set(true); }
  public void state5() { switch1.set(true);  switch2.set(false); switch3.set(true);  }
  public void state6() { switch1.set(false); switch2.set(true);  switch3.set(true); }
  public void state7() { switch1.set(true);  switch2.set(true);  switch3.set(true);  }

  public void runLights()
  {
    if(Robot.shooter.acquiredHatch)
      state1();
    else if(Robot.shooter.acquiredCargo)
      state2();
    else 
      state0();
  }

  @Override
  public void initDefaultCommand() {
    
    }
}