package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  //Subsytems
  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator elevator = new Elevator();
  public static Lights lights = new Lights();
  public static Intake intake = new Intake();
  public static OI oi;

  //Choosers
  Command autoCommand;
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<String> matrixChooser = new SendableChooser<>();

  //Vision Setup
  public static NetworkTable targetTable = NetworkTableInstance.getDefault().getTable("GRIP/targetReport");
  public static NetworkTableEntry targetEntryX = targetTable.getEntry("centerX");
  public static NetworkTableEntry targetEntryArea = targetTable.getEntry("area");
  public static double[] defaultValue = new double [0];
  public static double[] targetX;
  public static double[] targetArea;
  public static double x;
  public static double area;
  public static double distance;

  @Override
  public void robotInit() {
    oi = new OI();
    
    matrixChooser.addOption("", "");
    matrixChooser.addOption("That's How Mafia Works", "Mafia");
    matrixChooser.addOption("Sicko Mode or Mo Bamba?", "Sicko");
    matrixChooser.addOption("But Can It Do This???", "Chair");
    matrixChooser.addOption("Subscribe 2 Pewdiepie", "Pewds");

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

    //Vision Target Data Aquistion

    //X Coordinates
    targetX = targetEntryX.getDoubleArray(defaultValue);
    if(targetX.length < 1) {
    	x = 0;
    }
    else if(targetX.length == 1) {
    	x = targetX[0];
    }
    else {
    	x = (targetX[0] + targetX[1])/2;
    }

    //Area
    targetArea = targetEntryArea.getDoubleArray(defaultValue);
    if(targetArea.length < 1) {
    	area = 0;
    }
    else if(targetArea.length == 1) {
    	area = targetX[0];
    }
    else {
    	area = (targetArea[0] + targetArea[1])/2;
    }

    //Area to Distance Calculation
    distance = 57.7495 * Math.pow(Math.E, (-0.0000168466 * area));
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