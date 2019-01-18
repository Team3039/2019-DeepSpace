package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Kinematics;
import frc.robot.subsystems.Lights;

public class Robot extends TimedRobot {
  public static Drivetrain drivetrain = new Drivetrain();
  public static Kinematics kinematics = new Kinematics();
  public static Elevator elevator = new Elevator();
  public static Lights lights = new Lights();
  public static OI oi;

  Command autoCommand;
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  boolean red = true;

  @Override
  public void robotInit() {
    oi = new OI();
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", autoChooser);
    SmartDashboard.putBoolean("Red Alliance", true);

    UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
    usbCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 360, 60);
    usbCamera.setFPS(60);
    
    elevator.setupEncoder();
  }

  @Override
  public void robotPeriodic() {
    red = SmartDashboard.getBoolean("Red Alliance", true);
    if(red)
    {
      lights.setRedAlliance();
    }
    else
    {
      lights.setBlueAlliance();
    }
    lights.runLights();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autoCommand = autoChooser.getSelected();

    if (autoCommand != null) {
      autoCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("Elevator", elevator.getPosition());
  }

  @Override
  public void testPeriodic() {
  }
}
