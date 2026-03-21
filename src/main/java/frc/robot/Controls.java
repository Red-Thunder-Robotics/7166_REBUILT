package frc.robot;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Controls {
    public static final CommandXboxController driveController = new CommandXboxController(DRIVER_CONTROLLER);
    public static final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER);

    public static final Trigger manualTurretToggle = operatorController.rightStick();

    public static final Trigger resetGyroButton = driveController.start();
    public static final Trigger lockWheelsButton = driveController.a();
    public static final Trigger turboButton = driveController.b();

    public static final Trigger deployIntakeButton = driveController.rightBumper();
    public static final Trigger retractIntakeButton = driveController.leftBumper();
    public static final Trigger oscillateIntakeButton = driveController.rightTrigger();

    public static final Trigger reverseButton = operatorController.leftTrigger().or(driveController.leftTrigger());
    public static final Trigger hubButton = operatorController.rightTrigger();
    public static final Trigger allianceFeedButton = operatorController.x();

    public static final Trigger visionFail1 = operatorController.b();

    public static final Trigger rotationTargetBumpLeft = operatorController.leftBumper();
    public static final Trigger rotationTargetBumpRight = operatorController.rightBumper();

    // climber
    public static final Trigger climbUp = operatorController.y();
    public static final Trigger climbDown = operatorController.a();
}
