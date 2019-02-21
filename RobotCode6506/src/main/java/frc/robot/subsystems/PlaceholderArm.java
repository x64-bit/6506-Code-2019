/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * We have to make the encoder work so here's a quick
 * implementation of an arm w/o PID
 * 
 * peepeestyle.jpg
 */
public class PlaceholderArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Spark liftMotor = new Spark(RobotMap.ARM_MOTOR);

  public void setMotor(double val) {
    System.out.println("moving arm");
    liftMotor.set(val);
  }

  public void stop() {
    liftMotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
