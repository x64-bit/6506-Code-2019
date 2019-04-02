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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.OI;
//import com.ctre.phoenix.motorcontrol.pwm.VictorSPX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//NavX
import com.kauailabs.navx.frc.AHRS;
//import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

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
//NAVX.add IMPLEMENTS
public class Robot extends TimedRobot implements PIDOutput {
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

  //NavX.add
  PIDController turnController;

  AHRS ahrs;
  //NavX.add
  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system.  Note that the      */
  /* SmartDashboard in Test mode has support for helping you tune    */
  /* controllers by displaying a form where you can enter new P, I,  */
  /* and D constants and test the mechanism.                         */
  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double kToleranceDegrees = 2.0f;

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
    turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
    turnController.setInputRange(-180.0f,  180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);
    
    /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
    /* tuning of the Turn Controller's P, I and D coefficients.            */
    /* Typically, only the P value needs to be modified.                   */
    LiveWindow.addActuator("DriveSystem", "RotateController", turnController);

    Timer.delay(1.0);
    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    cam.setResolution(640, 480);        
}
//Ive gotten so far
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

  public void operatorControl() {
    myRobot.setSafetyEnabled(true);
      while (isOperatorControl() && isEnabled()) {
        boolean rotateToAngle = false;
      //replace this wiht the buttons used on our controller
        /*if ( stick.getRawButton(1)) {
          ahrs.reset();
        }*/
        if ( stick.getRawButton(2)) {
          turnController.setSetpoint(0.0f);
          rotateToAngle = true;
        } else if ( stick.getRawButton(3)) {
          turnController.setSetpoint(90.0f);
          rotateToAngle = true;
        } else if ( stick.getRawButton(4)) {
          turnController.setSetpoint(179.9f);
          rotateToAngle = true;
        } else if ( stick.getRawButton(5)) {
          turnController.setSetpoint(-90.0f);
          rotateToAngle = true;
        }
        double currentRotationRate;
        if ( rotateToAngle ) {
          turnController.enable();
          currentRotationRate = rotateToAngleRate;
        } else {
          turnController.disable();
         currentRotationRate = stick.getTwist();
        }
        try {
          /* Use the joystick X axis for lateral movement,          */
          /* Y axis for forward movement, and the current           */
          /* calculated rotation rate (or joystick Z axis),         */
          /* depending upon whether "rotate to angle" is active.    */
          myRobot.driveCartesian(stick.getX(), stick.getY(), 
                                       currentRotationRate, ahrs.getAngle());
        } catch( RuntimeException ex ) {
          DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
        }
        
        Timer.delay(0.005);		// wait for a motor update time
        }
    }
  }
}