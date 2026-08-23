package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
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

    default void updateInputs(IndexerIOInputs inputs) {}

    default void indexerVelocity(AngularVelocity velocity) {}

    default void indexerStop() {}

    default void topRollerVelocity(AngularVelocity velocity) {}

    default void topRollerStop() {}

    default void lowerKickerVelocity(AngularVelocity velocity) {}

    default void lowerKickerStop() {}
}
