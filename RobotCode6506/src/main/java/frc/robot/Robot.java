/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import frc.robot.commands.*;
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;
//import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.OI;
//import com.ctre.phoenix.motorcontrol.pwm.VictorSPX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

 
/**
 * Where everything runs.
 */
public class Robot extends TimedRobot {
  // placeholder for reference
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  // subsystems
  public static Spintake spinboi = new Spintake();
  public static PlaceholderArm arm = new PlaceholderArm();
  public static OI oi;
  // drive motors
  Spark driveLeft = new Spark(RobotMap.DRIVE_MOTORS_L);
  Spark driveRight = new Spark(RobotMap.DRIVE_MOTORS_R);
  // drivetrain
  public DifferentialDrive moveDude = new DifferentialDrive(driveLeft, driveRight);
  // scheduling and such
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  //public double x = 0;
  //public double y = 0;
  //Class Contructor for NavX Board (place in class contrucor for robot.java)
  // I'm sorry Noah -Anjo
  //That be ok
  //For some reason the Example shows this in a try/except block
  //help
  AHRS ahrs;
  // DO NOT TOUCH
  public Robot() {
    //stick = new Joystick(0);
    try {
  /***********************************************************************
   * navX-MXP:
   * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
   * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
   * 
   * navX-Micro:
   * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
   * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
   * 
   * Multiple navX-model devices on a single robot are supported.
   ************************************************************************/
        ahrs = new AHRS(SPI.Port.kMXP);
        //ahrs = new AHRS(SPI.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
        ahrs.enableLogging(true);
    } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    Timer.delay(1.0);
    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    cam.setResolution(640, 480);        
}
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    /**
     * arcadeDrive.drive(inputthing,inputthing) or something
     */
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //https://i.imgur.com/1FcKXFz.gif
    Scheduler.getInstance().run();
    //If you touch u go commit neck rope @ self & toaster bath @ self
    /*
    DO NOT TOUCH I SWEAR TO  IF YOU TOUCH THIS
    IT WILL BREAK I AM SERIOUS DO NOT TOUCH DO NOT
    BREAK PLEASE NO DO NOT TOUCH moveDude
    vvvvvvvvvvvvvvvv    
    */
    moveDude.arcadeDrive(-0.65 * OI.getLeftJoyY(), 0.65 * OI.getLeftJoyX());  //sudo touch moveDude
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}