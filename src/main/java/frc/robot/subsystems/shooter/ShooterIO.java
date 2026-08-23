package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        // flywheel
        double flywheelTargetVelocityRPS;

        double flywheelMotorTopLeftVelocityRPS;
        double flywheelMotorTopLeftCurrentAmps;

        double flywheelMotorBottomLeftVelocityRPS;
        double flywheelMotorBottomLeftCurrentAmps;

        double flywheelMotorTopRightVelocityRPS;
        double flywheelMotorTopRightCurrentAmps;

        double flywheelMotorBottomRightVelocityRPS;
        double flywheelMotorBottomRightCurrentAmps;

        double flywheelTotalCurrentAmps;

        // hood
        double hoodTargetPositionRotations;
        double hoodTargetPositionDegrees;

        double hoodPositionRotations;
        double hoodPositionDegrees;

        double hoodMotorDutyCycle;
        double hoodMotorVelocityRPS;
        double hoodMotorCurrentAmps;

        // kicker
        double upperKickerTargetVelocityRPS;

        double upperKickerMotorDutyCycle;
        double upperKickerMotorVelocityRPS;
        double upperKickerMotorCurrentAmps;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void idle() {}

    default void flywheelVelocity(AngularVelocity velocity) {}

    default void flywheelStop() {}

    default void hoodAngle(double position) {}

    default void hoodAngle(Angle angle) {}

    default void hoodStop() {}

    default void hoodZeroingDrive() {}

    default void hoodZero() {}

    default boolean hoodCanZero() {
        return true;
    }

    default void upperKickerVelocity(AngularVelocity velocity) {}

    default void upperKickerStop() {}
}
