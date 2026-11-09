package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ZeroingCommand;
import frc.robot.state_machine.IntakeState;
import frc.robot.state_machine.StateMachine;

/** Worked example: requests -> measured-state transitions -> complete outputs. */
public final class GroundIntakeSubsystem extends SubsystemBase {
    public enum WantedState {
        STOW, DEPLOY, INTAKE, REVERSE_HOME, REVERSE_DEPLOYED, OSCILLATE_OFF, OSCILLATE_ON
    }

    public enum SystemState {
        DISABLED, ZEROING,
        RETURNING_HOME, STOWING, STOWED,
        DEPLOYING, DEPLOYED, INTAKING,
        HOMING_FOR_REVERSE, REVERSING_HOME,
        DEPLOYING_FOR_REVERSE, REVERSING_DEPLOYED,
        OSCILLATING_OFF, OSCILLATING_ON
    }

    public static GroundIntakeSubsystem instance = null;
    private final GroundIntakeIO m_io;
    private final BooleanSupplier m_isShooting;
    private final GroundIntakeIOInputsAutoLogged m_inputs = new GroundIntakeIOInputsAutoLogged();
    private WantedState m_wantedState = WantedState.STOW;
    private SystemState m_systemState = SystemState.DISABLED;
    private final Timer m_pushTimer = new Timer();
    private enum PushStep { OutOne, InThree }
    private PushStep m_pushStep = PushStep.OutOne;

    public GroundIntakeSubsystem(GroundIntakeIO io) {
        this(io, StateMachine::isShooting);
    }

    GroundIntakeSubsystem(GroundIntakeIO io, BooleanSupplier isShooting) {
        instance = this;
        m_io = io;
        m_isShooting = isShooting;
    }

    /** Requests never write hardware. The next periodic cycle applies them. */
    public void setWantedState(WantedState wantedState) {
        m_wantedState = Objects.requireNonNull(wantedState);
    }

    public WantedState getWantedState() { return m_wantedState; }
    public SystemState getSystemState() { return m_systemState; }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("GroundIntake", m_inputs);

        if (DriverStation.isDisabled()) {
            // Clear requests so enabling cannot restart a previously running roller.
            StateMachine.setIntakeState(IntakeState.HomeOff);
            m_pushTimer.stop();
            m_systemState = SystemState.DISABLED;
        } else if (ZeroingCommand.isSubsystemZeroing(this)) {
            StateMachine.setIntakeState(IntakeState.HomeOff);
            m_pushTimer.stop();
            m_systemState = SystemState.ZEROING;
        } else {
            updatePushRequest();
            m_systemState = determineSystemState();
        }

        applyOutputs();
        Logger.recordOutput("GroundIntake/WantedState", m_wantedState);
        Logger.recordOutput("GroundIntake/SystemState", m_systemState);
        Logger.recordOutput("GroundIntake/AtHome", isIntakeHome());
        Logger.recordOutput("GroundIntake/AtDeployed", isAtDeployedPosition());
    }

    private SystemState determineSystemState() {
        return switch (m_wantedState) {
            case STOW -> {
                if (isIntakeHome()) yield SystemState.STOWED;
                // Continue pulling fuel inward while retracting from an out state.
                // Startup, zeroing and a home reverse request return with rollers off.
                yield switch (m_systemState) {
                    case DISABLED, ZEROING, RETURNING_HOME, STOWED,
                         HOMING_FOR_REVERSE, REVERSING_HOME -> SystemState.RETURNING_HOME;
                    default -> SystemState.STOWING;
                };
            }
            case DEPLOY -> isAtDeployedPosition() ? SystemState.DEPLOYED : SystemState.DEPLOYING;
            // Preserve RTR's immediate forward rollers during deployment.
            case INTAKE -> SystemState.INTAKING;
            case REVERSE_HOME -> isIntakeHome()
                ? SystemState.REVERSING_HOME : SystemState.HOMING_FOR_REVERSE;
            case REVERSE_DEPLOYED -> isAtDeployedPosition()
                ? SystemState.REVERSING_DEPLOYED : SystemState.DEPLOYING_FOR_REVERSE;
            case OSCILLATE_OFF -> SystemState.OSCILLATING_OFF;
            case OSCILLATE_ON -> SystemState.OSCILLATING_ON;
        };
    }

    private void applyOutputs() {
        switch (m_systemState) {
            case DISABLED -> {
                m_io.actuatorStop();
                m_io.rollerStop();
            }
            case ZEROING -> {
                // ZeroingCommand owns the actuator until it ends.
                m_io.rollerStop();
            }
            case RETURNING_HOME, STOWED, HOMING_FOR_REVERSE -> command(actuatorPositionHome, 0);
            case STOWING -> command(actuatorPositionHome, 1);
            case DEPLOYING, DEPLOYED, DEPLOYING_FOR_REVERSE -> command(actuatorPositionDeployed, 0);
            case INTAKING -> command(actuatorPositionDeployed, 1);
            case REVERSING_HOME -> command(actuatorPositionHome, -1);
            case REVERSING_DEPLOYED -> command(actuatorPositionDeployed, -1);
            case OSCILLATING_OFF -> command(actuatorPositionOscillate, 0);
            case OSCILLATING_ON -> command(actuatorPositionOscillate, 1);
        }
    }

    /** Every active state specifies both the actuator target and roller output. */
    private void command(Distance position, int rollerDirection) {
        m_io.setActuatorPosition(position);
        if (rollerDirection == 0) {
            m_io.rollerStop();
        } else if (rollerOutputUsesVelocityControl) {
            m_io.rollerVelocity(rollerDirection > 0 ? rollerOutputVelocity : rollerOutputVelocityReverse);
        } else {
            m_io.rollerCurrent(rollerDirection > 0 ? rollerOutputCurrent : rollerOutputCurrentReverse);
        }
    }

    private boolean atPosition(Distance position) {
        return Math.abs(m_inputs.actuatorPositionInches - position.in(Inches)) < 0.5d;
    }

    public boolean isAtDeployedPosition() { return atPosition(actuatorPositionDeployed); }
    public boolean isIntakeHome() { return atPosition(actuatorPositionHome); }
    public boolean areRollersStopped() { return Math.abs(m_inputs.rightRollerMotorVelocityRPS) < 10d; }

    public Command zeroMechanisms(boolean skipDrive) {
        return skipDrive
            ? Commands.runOnce(m_io::actuatorZero, this)
            : new ZeroingCommand(this, m_io::actuatorZeroingDrive, m_io::actuatorCanZero,
                m_io::actuatorStop, m_io::actuatorZero);
    }

    // Existing shooting coordination remains a request producer during this migration.
    private void updatePushRequest() {
        final boolean isTeleop = DriverStation.isTeleop();
        final boolean shouldPush = m_isShooting.getAsBoolean();
        if (m_pushTimer.isRunning()) {
            if (shouldPush) {
                final boolean advance = m_pushTimer.advanceIfElapsed(isTeleop ? pushFrequencySeconds : pushFrequencySecondsAuto);
                switch (m_pushStep) {
                    case OutOne:
                        if (isAtDeployedPosition()) {
                            StateMachine.setIntakeState(IntakeState.DeployedOn);
                        }
                        if (advance) m_pushStep = PushStep.InThree;
                        break;
                    case InThree:
                        StateMachine.setIntakeState(IntakeState.HomeOff);
                        break;
                }
            } else {
                m_pushTimer.stop();
            }
        } else if (shouldPush) {
            m_pushTimer.restart();
            m_pushStep = PushStep.OutOne;
        }
    }
}
