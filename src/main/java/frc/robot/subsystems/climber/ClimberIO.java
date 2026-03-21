package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        double targetPositionRotations;
        double targetPositionInches;

        double positionRotations;
        double positionInches;

        double velocityRPS;
        double currentAmps;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setPosition(double position) {}
    public default void stop() {}
}
