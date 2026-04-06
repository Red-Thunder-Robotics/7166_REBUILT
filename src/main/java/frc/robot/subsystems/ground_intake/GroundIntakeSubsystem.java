package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.state_machine.IntakeState;
import frc.robot.state_machine.ShooterState;
import frc.robot.state_machine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ConditionWaiter;

public final class GroundIntakeSubsystem extends SubsystemBase {
    public static GroundIntakeSubsystem instance = null;

    private final GroundIntakeIO m_io;
    private final GroundIntakeIOInputsAutoLogged m_inputs = new GroundIntakeIOInputsAutoLogged();

    private ConditionWaiter m_startRollerWaiter = new ConditionWaiter(this::isAtDeployedPosition);
    private ConditionWaiter m_reverseRollerWaiter = new ConditionWaiter(this::isAtDeployedPosition);
    private ConditionWaiter m_deployWaiter = new ConditionWaiter(this::areRollersStopped);
    private ConditionWaiter m_retractWaiter = new ConditionWaiter(this::areRollersStopped);

    private boolean m_rollerForward = false;
    private boolean m_rollerReverse = false;

    private static final double oscillateFrequencySeconds = 0.7d;
    private static final double oscillateStartDelaySeconds = 1d; // 1
    private static final int oscillateCountStopThreshold = 6;
    private static final boolean oscillateStopAfterCounInTeleop = false;
    private Timer m_oscillatorTimer = new Timer();
    private int m_oscillateCount = 0;

    public GroundIntakeSubsystem(GroundIntakeIO io) {
        instance = this;

        m_io = io;
    }

    @SuppressWarnings("unused")
    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        
        Logger.processInputs("GroundIntake", m_inputs);

        if (m_startRollerWaiter.process())
            startRoller();
        if (m_reverseRollerWaiter.process())
            reverseRoller();
        if (m_deployWaiter.process())
            deploy();
        if (m_retractWaiter.process())
            retract();

        if (m_rollerForward) {
            final double speedsX = Drive.instance.getChassisSpeeds().vxMetersPerSecond;
            AngularVelocity rollerOutput = rollerOutputVelocity;

            /*
             * goal is slow down rollers as you drive robot-relative forward.
             * chassis speed multiplied by 60 -> meters per minute
             * then divide by roller circumference -> rpm offset
             * substract rollerOutput minus rpm offset with a certain floor
             */
            if (rollerOutputVelocityUsesChassisSpeeds && DriverStation.isTeleopEnabled()) {
                if (speedsX > 0d) {
                    final double rpmOffset = speedsX * 60d / rollerCircumference.in(Meters);
                    double newRPM = rollerOutput.in(RPM) - rpmOffset;
                    newRPM = Math.max(newRPM, rollerOutputVelocityMinimum.in(RPM));
                    rollerOutput = RPM.of(newRPM);
                }
            } else
                rollerOutput = rollerOutputVelocityAuto;

            m_io.rollerVelocity(rollerOutput);
        } else if (m_rollerReverse)
            m_io.rollerVelocity(rollerOutputVelocityReverse);

        final boolean isTeleop = DriverStation.isTeleop();
        // final boolean shouldOscillate = StateMachine.wantsToShoot() && (isTeleop ? Controls.oscillateIntakeButton.getAsBoolean() : true);
        final boolean shouldOscillate = StateMachine.wantsToShoot() && !isTeleop;

        Logger.recordOutput("Intake/ShouldOscillate", shouldOscillate);
        if (m_oscillatorTimer.isRunning()) {
            if (shouldOscillate) {
                final boolean waiting = isTeleop ? false : m_oscillateCount < 1 && (m_oscillatorTimer.get() < oscillateStartDelaySeconds);
                if (!waiting && m_oscillatorTimer.advanceIfElapsed(oscillateFrequencySeconds)) {
                    final boolean skipStop = oscillateStopAfterCounInTeleop ? false : isTeleop;
                    if (skipStop || m_oscillateCount < oscillateCountStopThreshold) {
                        StateMachine.setIntakeState(StateMachine.getIntakeState() == IntakeState.DeployedOff ? IntakeState.OscillateOff : IntakeState.DeployedOff);
                        m_oscillateCount += 1;
                    } else
                        StateMachine.setIntakeState(IntakeState.HomeOff);
                }
            } else
                m_oscillatorTimer.stop();
        } else if (shouldOscillate) {
            m_oscillatorTimer.reset();
            m_oscillatorTimer.start();
            m_oscillateCount = 0;
        }
    }

    public boolean isAtDeployedPosition() {
        return m_inputs.isDeployed && Math.abs(m_inputs.actuatorPositionDegrees - m_inputs.targetActuatorPositionDegrees) < 10d;
    }
    public boolean areRollersStopped() {
        return Math.abs(m_inputs.rollerMotorVelocityRPS) < 10d;
    }

    public void stateUpdate(IntakeState intakeState, IntakeState oldIntakeState) {
        switch (intakeState) {
            case HomeOff:
                stopRoller();
                if (oldIntakeState.isOut())
                    m_retractWaiter.activate();
                else
                    retract();
                break;
            case HomeReverse:
                retract();
                if (oldIntakeState.isOut()) {
                    stopRoller();
                    m_reverseRollerWaiter.activate();
                } else
                    reverseRoller();
                break;
            case DeployedOff:
                stopRoller();
                deploy();
                break;
            case DeployedOn:
                deploy();
                // if (oldIntakeState.isDeployed())
                //     startRoller();
                // else
                //     m_startRollerWaiter.activate();
                startRoller();
                break;
            case DeployedReverse:
                deploy();
                if (oldIntakeState.isDeployed())
                    reverseRoller();
                else
                    m_reverseRollerWaiter.activate();
                break;
            case OscillateOff:
                deployOscillate();
                stopRoller();
                break;
            case OscillateOn:
                deployOscillate();
                stopRoller();
                m_io.rollerVelocity(rollerOutputVelocityHalfway);
                break;
        }
    }

    // private void setIdle() {
    //     m_io.idle();
    // }

    private void startRoller() {
        // m_io.rollerVelocity(rollerOutputVelocity);
        m_rollerForward = true;
    }
    private void reverseRoller() {
        // m_io.rollerVelocity(rollerOutputVelocityReverse);
        m_rollerReverse = true;
    }
    public void stopRoller() {
        m_rollerForward = false;
        m_rollerReverse = false;
        m_io.rollerStop();
    }

    public void deploy() {
        m_io.setActuatorPosition(actuatorPositionDeployed);
    }
    public void deployOscillate() {
        m_io.setActuatorPosition(actuatorPositionOscillate);
    }
    public void retract() {
        m_io.setActuatorPosition(actuatorPositionHome);
    }
}
