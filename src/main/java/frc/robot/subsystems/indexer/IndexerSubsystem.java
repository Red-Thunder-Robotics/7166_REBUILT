package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machine.StateMachine;

public final class IndexerSubsystem extends SubsystemBase {
    public static IndexerSubsystem instance = null;

    private final IndexerIO m_io;
    private final IndexerIOInputsAutoLogged m_inputs = new IndexerIOInputsAutoLogged();

    private boolean m_isFeeding;

    public IndexerSubsystem(IndexerIO io) {
        instance = this;

        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Indexer", m_inputs);

        boolean isFeeding = false;

        // if (StateMachine.getWantsReverseIndexerBeforeShoot()) {
        //     m_io.indexerVelocity(indexerOutputVelocityReverse);
        //     m_io.topRollerVelocity(topRollerVelocityReverse);
        // } else
        if (StateMachine.getIndexerShooterStuff()) {
            m_io.indexerVelocity(indexerOutputVelocity);
            m_io.topRollerVelocity(topRollerVelocity);
        }
        {
            switch (StateMachine.getShooterState()) {
                case Idle:
                    setIdle();
                    break;
                case Shooting:
                    if (StateMachine.shouldIndex()) {
                        isFeeding = true;
                        // m_io.indexerVelocity(indexerOutputVelocity);
                        // m_io.topRollerVelocity(topRollerVelocity);
                        m_io.lowerKickerVelocity(lowerKickerVelocity);
                    } else
                        setIdle();
                    break;
                case Reversing:
                    m_io.indexerVelocity(indexerOutputVelocityReverse);
                    m_io.topRollerVelocity(topRollerVelocityReverse);
                    m_io.lowerKickerVelocity(lowerKickerVelocityReverse);
                    break;
            }
        }

        m_isFeeding = isFeeding;
    }

    @AutoLogOutput(key="IndexerIsFeeding")
    public boolean getIsFeeding() {
        return m_isFeeding;
    }

    public void setIdle() {
        m_io.indexerStop();
        m_io.topRollerStop();
        m_io.lowerKickerStop();
    }
}
