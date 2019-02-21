/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;

/**
 * Broken but not entirely sure why. Maybe the encoder?
 */
public class Tentacle extends PIDSubsystem {
  /**
   * help
   */
  private Spark armMotor;
  private Encoder enc;

  public Tentacle() {
    // Intert a subsystem name and PID values here
    super("Arm", 2, 0.1, 0);
    armMotor = new Spark(RobotMap.ARM_MOTOR);
    enc = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    // this is where I'd put an initDefaultCommand
    // IF I HAD ONE
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return enc.getDistance();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    System.out.println("I AM LIFTING DO THE THING PLEASEEE");
    // armMotor.pidWrite(output);
  }

  public void setMotor(double speed) {
    usePIDOutput(speed);
  }
}

