package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.robot.util.CommandUtil.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machine.LauncherTarget;
import frc.robot.state_machine.StateMachine;
import frc.robot.util.ConversionUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public final class TurretSubsystem extends SubsystemBase {
    public static TurretSubsystem instance = null;

    private final TurretIO m_io;
    private final TurretIOInputsAutoLogged m_inputs = new TurretIOInputsAutoLogged();

    public TurretSubsystem(TurretIO io) {
        instance = this;

        m_io = io;
    }

    @AutoLogOutput(key = "TurretDebugPose")
    private Pose3d m_debugPose = new Pose3d();

    public Pose3d getDebugPose() {
        return m_debugPose;
    }

    public Rotation3d getFieldRotation() {
        return m_debugPose.getRotation();
    }

    @AutoLogOutput(key = "TurretShouldIndex")
    public boolean shouldIndex() {
        return Math.abs(m_inputs.targetPositionDegrees - m_inputs.positionDegrees) <= shouldIndexThresholdDegrees;
    }

    @AutoLogOutput(key = "TurretManualEnabled")
    private boolean m_manualEnabled = false;

    private Angle m_manualTarget;
    private DoubleSupplier m_manualSupplier = () -> 0d;

    // t2
    // private static final double lookAheadTime = 0.05d;

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Turret", m_inputs);

        if (m_manualEnabled) {
            m_manualTarget = m_manualTarget.plus(Degrees.of(m_manualSupplier.getAsDouble()));
            setAngle(m_manualTarget);
        } else {
            StateMachine.getLauncherTargetPose().ifPresent(this::target3dPose);
        }

        m_debugPose = new Pose3d(
                0,
                0,
                Units.inchesToMeters(30d),
                new Rotation3d(0, 0, Units.degreesToRadians(m_inputs.positionDegrees)));
    }

    public boolean getManualEnabled() {
        return m_manualEnabled;
    }

    public Command toggleManualModeCommand() {
        return cmdName(
                runOnce(() -> {
                    m_manualEnabled = !m_manualEnabled;
                    if (m_manualEnabled) m_manualTarget = getAngle();
                }),
                "TurretManualMode");
    }

    public void setManualSupplier(DoubleSupplier supplier) {
        m_manualSupplier = supplier;
    }

    public Angle getAngle() {
        return Degrees.of(m_inputs.positionDegrees);
    }

    public void setAngle(Angle measurement) {
        m_io.setPosition(ConversionUtil.angleToMechanismPosition(measurement));
    }

    public void stateUpdate(LauncherTarget launcherTarget) {
        m_manualEnabled = false;
        switch (launcherTarget) {
            case Idle:
                m_io.idle();
                break;
            case HubTracking:
                break;
            case AllianceFeed:
                break;
        }
    }

    private void target3dPose(Translation3d targetPose) {
        setAngle(StateMachine.getRobotRotationalGoal(targetPose));
    }
}
