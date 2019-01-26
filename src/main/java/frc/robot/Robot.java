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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class Robot extends TimedRobot {
  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator elevator = new Elevator();
  public static Lights lights = new Lights();
  public static Intake intake = new Intake();
  public static OI oi;

  Command autoCommand;
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<String> matrixChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    oi = new OI();
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    matrixChooser.addOption("", "");
    matrixChooser.addOption("That's How Mafia Works", "Mafia");
    matrixChooser.addOption("Sicko Mode or Mo Bamba?", "Sicko");
    matrixChooser.addOption("But Can It Do This???", "But");
    matrixChooser.addOption("Subscribe 2 Pewdiepie", "Sub");

    SmartDashboard.putData("Auto mode", autoChooser);
    SmartDashboard.putBoolean("Red Alliance", true);
    SmartDashboard.putData("End game text", matrixChooser);

    UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
    usbCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 180, 60);
    
    elevator.setupEncoder();
  }

  @Override
  public void robotPeriodic() {
    
    boolean red = SmartDashboard.getBoolean("Red Alliance", true);
    if(red)
    {
      lights.setRedAlliance();
    }
    else
    {
      lights.setBlueAlliance();
    }
    //System.out.println(lights.getAlliance());
    String selected = matrixChooser.getSelected();
    if(selected != null && !selected.equals(""))
    {
      //System.out.println(selected);
      lights.setMatrixText(selected);
    }
    else
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