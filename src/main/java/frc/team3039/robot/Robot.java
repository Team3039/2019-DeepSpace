package frc.team3039.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.robot.subsystems.Climber;
import frc.team3039.robot.subsystems.Drive;
import frc.team3039.robot.subsystems.Elevator;
import frc.team3039.robot.subsystems.Intake;
import frc.team3039.robot.subsystems.Lights;
import frc.team3039.robot.subsystems.Shooter; 

public class Robot extends TimedRobot {
  public static OI oi;

  //Subsytems
  public static final Drive drive = Drive.getInstance();
  public static final Elevator elevator = Elevator.getInstance();
  public static final Intake intake = Intake.getInstance();
  public static Lights lights = Lights.getInstance();
  public static Shooter shooter = Shooter.getInstance();
  public static Climber climber = Climber.getInstance();
  

  //Choosers
  Command autoCommand;
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  //Vision Information
  public static double targetValid; //Whether the limelight has any valid targets (0 or 1)
  public static double targetX; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  public static double targetY; //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  public static double targetArea; //Target Area (0% of image to 100% of image)
  public static double targetSkew; //Skew or rotation (-90 degrees to 0 degrees)

  @Override
  public void robotInit() {
    oi = OI.getInstance();

    SmartDashboard.putData("Auto mode", autoChooser);
    SmartDashboard.putBoolean("Red Alliance", true);

    UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
    usbCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 180, 60);
    
    elevator.setupEncoder();
    Robot.drive.setDriverCamMode();

    System.out.println("Only True Led-Gends Will Know");
  }

  @Override
  public void robotPeriodic() {
    lights.runLights();
    if(SmartDashboard.getBoolean("Red Alliance", true)) {
      lights.setRedAlliance();
    }
    else {
        lights.setBlueAlliance();
    }

    //Gather Vision Information
    targetValid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    targetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    targetSkew = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

    // SmartDashboard.putNumber("Valid Target", targetValid);
    // SmartDashboard.putNumber("Target X", targetX);
    // SmartDashboard.putNumber("Target Y", targetY);
    // SmartDashboard.putNumber("Target Area", targetArea);
    // SmartDashboard.putNumber("Target Skew", targetSkew);
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
    Robot.drive.setDriverCamMode();
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

    SmartDashboard.putNumber("Position", Robot.elevator.getPosition());
    SmartDashboard.putNumber("Velocity", Robot.elevator.getVelocity());
    // SmartDashboard.putNumber("Error", Robot.elevator.elevatorA.getClosedLoopError());
    // System.out.println("\n Hatch Pres  sure is " + Robot.shooter.getHatchStatus());
    // System.out.println("\n  Raising " + Robot.elevator.isRaising);
    // System.out.println("\n  Lowering " + Robot.elevator.isLowering);
    // System.out.println("\n   " + Robot.shooter.acquiredCargo);
    // System.out.println("\n " + Robot.elevator.elevatorA.getAppliedOutput());
    // System.out.println("\n" + Robot.elevator.getVelocity());
    // System.out.println("\n" + Robot.elevator.getPosition());
  }

  @Override
  public void testPeriodic() {
  }
}