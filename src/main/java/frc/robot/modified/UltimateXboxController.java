package frc.robot.modified;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class UltimateXboxController extends CommandXboxController {
    public UltimateXboxController(int id) {
        super(id);
    }

    public Trigger paddleTopLeft() {
        return povDown().or(povDownLeft()).or(povDownRight());
    }
    public Trigger paddleBottomLeft() {
        return povLeft().or(povUpLeft()).or(povDownLeft());
    }
    public Trigger paddleTopRight() {
        return povRight().or(povUpRight()).or(povDownRight());
    }
    public Trigger paddleBottomRight() {
        return povUp().or(povUpLeft()).or(povUpRight());
    }
}
