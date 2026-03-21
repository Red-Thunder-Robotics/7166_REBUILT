package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        // left
        double leftTargetPositionRotations;
        double leftTargetPositionInches;

        double leftPositionRotations;
        double leftPositionInches;

        double leftDutyCycle;
        double leftVelocityRPS;
        double leftCurrentAmps;

        // right
        double rightTargetPositionRotations;
        double rightTargetPositionInches;

        double rightPositionRotations;
        double rightPositionInches;

        double rightDutyCycle;
        double rightVelocityRPS;
        double rightCurrentAmps;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void leftIdle() {}
    public default void rightIdle() {}

    public default void setLeftPosition(double position) {}
    public default void setRightPosition(double position) {}
}
