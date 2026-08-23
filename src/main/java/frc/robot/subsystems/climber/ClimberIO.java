package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        double targetPositionRotations;
        double targetPositionInches;

        double positionRotations;
        double positionInches;

        double velocityRPS;
        double currentAmps;
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setPosition(double position) {}

    default void stop() {}
}
