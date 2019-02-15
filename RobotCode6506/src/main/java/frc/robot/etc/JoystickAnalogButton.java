/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.etc;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Reads analog inputs as a button.
 */
public class JoystickAnalogButton extends Button {

    XboxController controller;
    int axisNumber;
    private final double THRESHOLD = 0.25;

    public JoystickAnalogButton(XboxController controller, int axis) {
        this.controller = controller;
        axisNumber = axis;
    }

    public boolean get() {
        if (controller.getRawAxis(axisNumber) > THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }
}
