package frc.robot.subsystems.ground_intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
    class GroundIntakeIOInputs {
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

    default void updateInputs(GroundIntakeIOInputs inputs) {}

    default void idle() {}

    default void setActuatorPosition(double position) {}

    default void actuatorStop() {}

    default void rollerVelocity(AngularVelocity velocity) {}

    default void rollerCurrent(Current current) {}

    default void rollerStop() {}

    default void actuatorZeroingDrive() {}

    default void actuatorZero() {}

    default boolean actuatorCanZero() {
        return true;
    }
}
