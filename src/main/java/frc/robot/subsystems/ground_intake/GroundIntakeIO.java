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

        double rollerMotorDutyCycle;
        double rollerMotorVelocityRPS;
        double rollerMotorCurrentAmps;
    }

    public default void updateInputs(GroundIntakeIOInputs inputs) {}

    public default void idle() {}
    
    public default void setActuatorPosition(double position) {}
    public default void rollerVelocity(AngularVelocity velocity) {}
    public default void rollerStop() {}
}
