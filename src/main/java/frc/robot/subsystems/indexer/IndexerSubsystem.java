package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machine.RobotEvent;
import frc.robot.state_machine.StateMachine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public final class IndexerSubsystem extends SubsystemBase {
    public static IndexerSubsystem instance = null;

    private final IndexerIO m_io;
    private final IndexerIOInputsAutoLogged m_inputs = new IndexerIOInputsAutoLogged();

    private boolean m_isFeeding;

    private Timer m_reverseTimer = new Timer();

    public IndexerSubsystem(IndexerIO io) {
        instance = this;

        m_io = io;

        RobotEvent.OnShootingStart.addListener(() -> {
            m_io.topRollerVelocity(topRollerVelocityReverse);
            m_reverseTimer.restart();
        });
        RobotEvent.OnShootingEnd.addListener(() -> {
            m_reverseTimer.stop();
        });
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
        // if (StateMachine.getIndexerShooterStuff()) {
        //     m_io.indexerVelocity(indexerOutputVelocity);
        //     m_io.topRollerVelocity(topRollerVelocity);
        // }
        {
            switch (StateMachine.getShooterState()) {
                case Idle:
                    setIdle();
                    break;
                case Shooting:
                    if (getReverseIsDone()) {
                        if (StateMachine.shouldIndex()) {
                            isFeeding = true;
                            m_io.indexerVelocity(indexerOutputVelocity);
                            m_io.topRollerVelocity(topRollerVelocity);
                        } else setIdle();
                        m_io.lowerKickerVelocity(lowerKickerVelocity);
                    }
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

    @AutoLogOutput(key = "IndexerIsFeeding")
    public boolean getIsFeeding() {
        return m_isFeeding;
    }

    @AutoLogOutput(key = "IndexerPastThreshold")
    public boolean getIsPastThreshold() {
        final double lowerKickerTarget = m_inputs.lowerKickerVelocityRPS;
        if (lowerKickerTarget == 0d) return false;

        final boolean lowerKicker =
                Math.abs(lowerKickerTarget - m_inputs.lowerKickerTargetVelocityRPS) <= lowerKickerVelocityThresholdRPS;

        return lowerKicker;
    }

    @AutoLogOutput(key = "ReverseDone")
    public boolean getReverseIsDone() {
        return m_reverseTimer.hasElapsed(reverseTime);
    }

    public void setIdle() {
        m_io.indexerStop();
        m_io.topRollerStop();
        m_io.lowerKickerStop();
    }
}
