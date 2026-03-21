package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // flywheel
        double flywheelTargetVelocityRPS;

        double flywheelMotorLeftVelocityRPS;
        double flywheelMotorLeftCurrentAmps;

        double flywheelMotorBottomLeftVelocityRPS;
        double flywheelMotorBottomLeftCurrentAmps;

        double flywheelMotorMiddleLowerVelocityRPS;
        double flywheelMotorMiddleLowerCurrentAmps;

        double flywheelMotorRightVelocityRPS;
        double flywheelMotorRightCurrentAmps;

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

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void idle() {}

    public default void flywheelVelocity(AngularVelocity velocity) {}
    public default void flywheelStop() {}

    public default void hoodAngle(double position) {}
    public default void hoodAngle(Angle angle) {}

    public default void upperKickerVelocity(AngularVelocity velocity) {}
    public default void upperKickerStop() {}
}
