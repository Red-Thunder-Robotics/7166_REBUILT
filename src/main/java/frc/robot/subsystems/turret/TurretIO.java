package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        double targetPositionRotations;
        double targetPositionDegrees;

        double positionRotations;
        double positionDegrees;

        double dutyCycle;
        double velocityRPS;
        double currentAmps;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void idle() {}

    default void setPosition(double position) {}
}
