package frc.robot.subsystems.ground_intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs {
        boolean isDeployed;

        double targetActuatorPositionRotations;
        double targetActuatorPositionDegrees;

        double actuatorPositionRotations;
        double actuatorPositionDegrees;
        double actuatorMotorCurrentAmps;

        double rollerTargetVelocityRPS;
        double rollerTotalCurrentAmps;

        double rightRollerMotorDutyCycle;
        double rightRollerMotorVelocityRPS;
        double rightRollerMotorCurrentAmps;

        double leftRollerMotorDutyCycle;
        double leftRollerMotorVelocityRPS;
        double leftRollerMotorCurrentAmps;
    }

    public default void updateInputs(GroundIntakeIOInputs inputs) {}

    public default void idle() {}
    
    public default void setActuatorPosition(double position) {}
    public default void actuatorStop() {}

    public default void rollerVelocity(AngularVelocity velocity) {}
    public default void rollerStop() {}

    public default void actuatorZeroingDrive() {}
    public default void actuatorZero() {}
    public default boolean actuatorCanZero() { return true; }
}
