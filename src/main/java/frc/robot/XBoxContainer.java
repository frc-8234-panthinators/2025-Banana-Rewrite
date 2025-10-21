package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XBoxContainer {
    private XboxController controller = new XboxController(0);

    public Trigger coralIntake = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
    public Trigger coralOuttake = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);

    public Trigger alignRight = new Trigger(() -> controller.getRightTriggerAxis() > 0.1);
    public Trigger alignLeft = new Trigger(() -> controller.getLeftTriggerAxis() > 0.1);
    
    public Trigger dpadDownL1 = new Trigger(() -> controller.getPOV() == 180);
    public Trigger dpadLeftL2 = new Trigger(() -> controller.getPOV() == 270);
    public Trigger dpadRightL3 = new Trigger(() -> controller.getPOV() == 90);
    public Trigger dpadUpL4 = new Trigger(() -> controller.getPOV() == 0);
   
    public Trigger algaeIntake = new JoystickButton(controller, XboxController.Button.kA.value);
    public Trigger algaeOuttake = new JoystickButton(controller, XboxController.Button.kB.value);
    public Trigger cancel = new JoystickButton(controller, XboxController.Button.kX.value);
    public Trigger upperAlgae = new JoystickButton(controller, XboxController.Button.kY.value);

    public Trigger back = new JoystickButton(controller, XboxController.Button.kBack.value);
    public Trigger start = new JoystickButton(controller, XboxController.Button.kStart.value);

    public boolean getControllerXButton() {
        return(controller.getLeftStickButton());
    }

    public double driveX() {
        if (Math.abs(controller.getLeftX()) >= 0.1) {
            return(controller.getLeftX() * 0.2);
        } else {
            return(0);
        }
    }

    public double driveY() {
        if (Math.abs(controller.getLeftY()) >= 0.1) {
            return(controller.getLeftY() * 0.2);
        } else {
            return(0);
        }
    }

    public double rotate() {
        if (Math.abs(controller.getRightX()) >= 0.1) {
            return(controller.getRightX() * 0.5);
        } else {
            return(0);
        }
    }

    public double elevatorManual() {
        return controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    }

    public boolean runAlgae() {
        return controller.getAButton();
    }

    public boolean runCoral() {
        return controller.getBButton();
    }

    public boolean outtakeCoral() {
        return controller.getYButton();
    }
}
