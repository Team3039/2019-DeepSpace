
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class Lights extends Subsystem {

  public DigitalOutput switch1 = new DigitalOutput(RobotMap.switch1);
  public DigitalOutput switch2 = new DigitalOutput(RobotMap.switch2);
  public DigitalOutput switch3 = new DigitalOutput(RobotMap.switch3);
  public DigitalOutput switch4 = new DigitalOutput(RobotMap.switch4);

  public void state0() { switch3.set(false); switch2.set(false); switch1.set(false); }
  public void state1() { switch3.set(true);  switch2.set(false); switch1.set(true);  }
  public void state2() { switch3.set(false); switch2.set(true);  switch1.set(false); }
  public void state3() { switch1.set(false); switch2.set(true);  switch1.set(true);  }
  public void state4() { switch3.set(true);  switch2.set(false); switch1.set(false); }
  public void state5() { switch3.set(true);  switch2.set(false); switch1.set(true);  }
  public void state6() { switch3.set(true);  switch2.set(true);  switch1.set(false); }
  public void state7() { switch3.set(true);  switch2.set(true);  switch1.set(true);  }

  public void runLights()
  {
    if(Robot.shooter.acquiredHatch)
      state1();
    else if(Robot.shooter.acquiredCargo)
      state2();
    else 
      state0();
  }

  public void setMatrixText(String text)
  {
    if(text.equals("Mafia"))
      state4();
    else if(text.equals("Sicko"))
      state5();
    else if(text.equals("But"))
      state6();
    else if(text.equals("Sub"))
      state7();
    else
      runLights();
  }

  @Override
  public void initDefaultCommand() {
    runLights();
  }
}