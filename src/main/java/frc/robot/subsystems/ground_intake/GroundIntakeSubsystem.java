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
        HomeOff, HomeReverse, DeployedOff, DeployedOn, DeployedReverse, OscillateOff, OscillateOn
    }

    public enum SystemState {
        HomeOff, HomeReverse, DeployedOff, DeployedOn, DeployedReverse, OscillateOff, OscillateOn,
        // Extra output combinations needed while executing C3's requests.
        HomeOn, OscillateReverse, Disabled, Zeroing
    }

    public static GroundIntakeSubsystem instance = null;
    private final GroundIntakeIO m_io;
    private final BooleanSupplier m_isShooting;
    private final GroundIntakeIOInputsAutoLogged m_inputs = new GroundIntakeIOInputsAutoLogged();
    private WantedState m_wantedState = WantedState.HomeOff;
    private SystemState m_systemState = SystemState.Disabled;
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
            m_systemState = SystemState.Disabled;
        } else if (ZeroingCommand.isSubsystemZeroing(this)) {
            StateMachine.setIntakeState(IntakeState.HomeOff);
            m_pushTimer.stop();
            m_systemState = SystemState.Zeroing;
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
            case HomeOff -> {
                if (isIntakeHome()) yield SystemState.HomeOff;
                // C3 pulls fuel inward when returning from an out state.
                yield switch (m_systemState) {
                    case DeployedOff, DeployedOn, DeployedReverse,
                         OscillateOff, OscillateOn, OscillateReverse, HomeOn -> SystemState.HomeOn;
                    default -> SystemState.HomeOff;
                };
            }
            case HomeReverse -> isIntakeHome() ? SystemState.HomeReverse : SystemState.HomeOff;
            case DeployedOff -> SystemState.DeployedOff;
            // C3 runs the forward rollers immediately, including during deployment.
            case DeployedOn -> SystemState.DeployedOn;
            case DeployedReverse -> isAtDeployedPosition()
                ? SystemState.DeployedReverse : SystemState.DeployedOff;
            case OscillateOff -> SystemState.OscillateOff;
            case OscillateOn -> {
                // C3 starts forward at deployed; otherwise it retains the roller action.
                if (isAtDeployedPosition()) yield SystemState.OscillateOn;
                yield switch (m_systemState) {
                    case DeployedOn, HomeOn, OscillateOn -> SystemState.OscillateOn;
                    case HomeReverse, DeployedReverse, OscillateReverse -> SystemState.OscillateReverse;
                    default -> SystemState.OscillateOff;
                };
            }
        };
    }

    private void applyOutputs() {
        switch (m_systemState) {
            case Disabled -> {
                m_io.actuatorStop();
                stopRoller();
            }
            case Zeroing -> stopRoller(); // ZeroingCommand owns the actuator.
            case HomeOff -> { retract(); stopRoller(); }
            case HomeOn -> { retract(); startRoller(); }
            case HomeReverse -> { retract(); reverseRoller(); }
            case DeployedOff -> { deploy(); stopRoller(); }
            case DeployedOn -> { deploy(); startRoller(); }
            case DeployedReverse -> { deploy(); reverseRoller(); }
            case OscillateOff -> { deployOscillate(); stopRoller(); }
            case OscillateOn -> { deployOscillate(); startRoller(); }
            case OscillateReverse -> { deployOscillate(); reverseRoller(); }
        }
    }

    // C3 action names; only applyOutputs calls these, so each state owns all outputs.
    private void startRoller() {
        if (rollerOutputUsesVelocityControl) m_io.rollerVelocity(rollerOutputVelocity);
        else m_io.rollerCurrent(rollerOutputCurrent);
    }
    private void reverseRoller() {
        if (rollerOutputUsesVelocityControl) m_io.rollerVelocity(rollerOutputVelocityReverse);
        else m_io.rollerCurrent(rollerOutputCurrentReverse);
    }
    private void stopRoller() { m_io.rollerStop(); }
    private void deploy() { m_io.setActuatorPosition(actuatorPositionDeployed); }
    private void deployOscillate() { m_io.setActuatorPosition(actuatorPositionOscillate); }
    private void retract() { m_io.setActuatorPosition(actuatorPositionHome); }

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
