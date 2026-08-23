package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;

public final class IndexerIOSim implements IndexerIO {
    private double m_indexerTargetVelocity = 0d;
    private double m_indexerVelocity = 0d;

    private double m_topRollerTargetVelocity = 0d;
    private double m_topRollerVelocity = 0d;

    private double m_lowerKickerTargetVelocity = 0d;
    private double m_lowerKickerVelocity = 0d;

    private final PIDController m_indexerPID = new PIDController(0.5d, 0d, 0d);
    private final PIDController m_topRollerPID = new PIDController(0.5d, 0d, 0d);
    private final PIDController m_lowerKickerPID = new PIDController(0.5d, 0d, 0d);

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerTargetVelocityRPS = m_indexerTargetVelocity;
        m_indexerVelocity += m_indexerPID.calculate(m_indexerVelocity);
        inputs.indexerVelocityRPS = m_indexerVelocity;

        inputs.topRollerTargetVelocityRPS = m_topRollerTargetVelocity;
        m_topRollerVelocity += m_topRollerPID.calculate(m_topRollerVelocity);
        inputs.topRollerVelocityRPS = m_topRollerVelocity;

        inputs.lowerKickerTargetVelocityRPS = m_lowerKickerTargetVelocity;
        m_lowerKickerVelocity += m_lowerKickerPID.calculate(m_lowerKickerVelocity);
        inputs.lowerKickerVelocityRPS = m_lowerKickerVelocity;
    }

    @Override
    public void indexerVelocity(AngularVelocity velocity) {
        m_indexerTargetVelocity = velocity.in(RotationsPerSecond);
        m_indexerPID.setSetpoint(m_indexerTargetVelocity);
    }

    @Override
    public void indexerStop() {
        indexerVelocity(RotationsPerSecond.of(0d));
    }

    @Override
    public void topRollerVelocity(AngularVelocity velocity) {
        m_topRollerTargetVelocity = velocity.in(RotationsPerSecond);
        m_topRollerPID.setSetpoint(m_topRollerTargetVelocity);
    }

    @Override
    public void topRollerStop() {
        topRollerVelocity(RotationsPerSecond.of(0d));
    }

    @Override
    public void lowerKickerVelocity(AngularVelocity velocity) {
        m_lowerKickerTargetVelocity = velocity.in(RotationsPerSecond);
        m_lowerKickerPID.setSetpoint(m_lowerKickerTargetVelocity);
    }

    @Override
    public void lowerKickerStop() {
        lowerKickerVelocity(RotationsPerSecond.of(0d));
    }
}
