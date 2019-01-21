
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


public class Lights extends Subsystem {

  public DigitalOutput switch1 = new DigitalOutput(RobotMap.switch1);
  public DigitalOutput switch2 = new DigitalOutput(RobotMap.switch2);
  public DigitalOutput switch3 = new DigitalOutput(RobotMap.switch3);
  public DigitalOutput directionSwitch = new DigitalOutput(RobotMap.directionSwitch);

  public static boolean redAlliance = true;
  public static boolean hatchGrabbed = false;
  public static boolean cargoGrabbed = false;

  public void state0() { switch1.set(false); switch2.set(false); switch3.set(false);}
  public void state1() { switch1.set(true);  switch2.set(false); switch3.set(false);}
  public void state2() { switch1.set(false); switch2.set(true);  switch3.set(false);}
  public void state3() { switch1.set(true);  switch2.set(true);  switch3.set(false);}
  public void state4() { switch1.set(false); switch2.set(false); switch3.set(true); }
  public void state5() { switch1.set(true);  switch2.set(false); switch3.set(true); }
  public void state6() { switch1.set(false); switch2.set(true);  switch3.set(true); }
  public void state7() { switch1.set(true);  switch2.set(true);  switch3.set(true); }

  public void setDirectionForwards() { directionSwitch.set(false); }
  public void setDirectionBackWards() { directionSwitch.set(true); }

  public void setRedAlliance()
  {
    redAlliance = true;
  }

  public void setBlueAlliance()
  {
    redAlliance = false;
  }

  public void grabbedHatch()
  {
    hatchGrabbed = true;
    cargoGrabbed = false;
  }

  public void grabbedCargo()
  {
    hatchGrabbed = false;
    cargoGrabbed = true;
  }

  public void releasedGamePiece()
  {
    hatchGrabbed = false;
    cargoGrabbed = false;
  }

  public void runAlliance()
  {
    if(!hatchGrabbed && !cargoGrabbed)
    {
      if(redAlliance)
      {
        state0();
      }
      else
      {
        state1();
      }
    }
  }

  public void runLights()
  {
    if(hatchGrabbed)
    {
      state2();
    }
    else if(cargoGrabbed)
    {
      state3();
    }
    else 
    {
      runAlliance();
    }
  }

  @Override
  public void initDefaultCommand() {
    runAlliance();
    setDirectionForwards();
  }
}
