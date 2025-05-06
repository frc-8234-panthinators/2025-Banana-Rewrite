package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XBoxContainer {
    private XboxController controller = new XboxController(0);
    private XboxController otherController = new XboxController(1);
    public Trigger a = new JoystickButton(otherController, XboxController.Button.kA.value);
    public Trigger x = new JoystickButton(otherController, XboxController.Button.kX.value);
    public Trigger b = new JoystickButton(otherController, XboxController.Button.kB.value);

    public boolean getControllerXButton() {
        return(controller.getXButton());
    }

    public double driveX() {
        if (Math.abs(controller.getLeftX()) >= 0.1) {
            return(controller.getLeftX() * 2.5);
        } else {
            return(0);
        }
    }

    public double driveY() {
        if (Math.abs(controller.getLeftY()) >= 0.1) {
            return(controller.getLeftY() * 2.5);
        } else {
            return(0);
        }
    }

    public double rotate() {
        if (Math.abs(controller.getRightX()) >= 0.1) {
            return(controller.getRightX() * 2.5);
        } else {
            return(0);
        }
    }
}
