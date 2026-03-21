package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        double indexerTargetVelocityRPS;
        double indexerVelocityRPS;
        double indexerCurrentAmps;

        double topRollerTargetVelocityRPS;
        double topRollerVelocityRPS;
        double topRollerCurrentAmps;

        double lowerKickerTargetVelocityRPS;
        double lowerKickerVelocityRPS;
        double lowerKickerCurrentAmps;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void indexerVelocity(AngularVelocity velocity) {}
    public default void indexerStop() {}

    public default void topRollerVelocity(AngularVelocity velocity) {}
    public default void topRollerStop() {}

    public default void lowerKickerVelocity(AngularVelocity velocity) {}
    public default void lowerKickerStop() {}
}
