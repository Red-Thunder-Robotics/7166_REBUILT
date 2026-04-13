package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.allianceFeedParamsFar;
import static frc.robot.subsystems.shooter.ShooterConstants.allianceFeedParamsNeutralZone;
import static frc.robot.subsystems.shooter.ShooterConstants.flywheelRadiusBig;
import static frc.robot.subsystems.shooter.ShooterConstants.flywheelRadiusSmall;
import static frc.robot.subsystems.shooter.ShooterConstants.hoodAutoStowSeconds;
import static frc.robot.subsystems.shooter.ShooterConstants.hoodPositionHome;
import static frc.robot.subsystems.shooter.ShooterConstants.hubCenterParams;
import static frc.robot.subsystems.shooter.ShooterConstants.paramMap;
import static frc.robot.subsystems.shooter.ShooterConstants.shouldIndexFlywheelVelocityThresholdRPS;
import static frc.robot.subsystems.shooter.ShooterConstants.shouldIndexKickerVelocityThresholdRPS;
import static frc.robot.subsystems.shooter.ShooterConstants.timeOfFlightMap;
import static frc.robot.subsystems.shooter.ShooterConstants.upperKickerVelocity;
import static frc.robot.subsystems.shooter.ShooterConstants.upperKickerVelocityReverse;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.commands.ZeroingCommand;
import frc.robot.state_machine.LauncherTarget;
import frc.robot.state_machine.LiveConfig;
import frc.robot.state_machine.RobotEvent;
import frc.robot.state_machine.ShooterState;
import frc.robot.state_machine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.InterpolationShooterParams;

public final class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem instance = null;

    private final ShooterIO m_io;
    private final ShooterIOInputsAutoLogged m_inputs;

    private final Timer m_autoStowTimer = new Timer();
    private final Timer m_kickerStartupTimer = new Timer();
    private boolean m_mechanismsPastThreshold = false;

    public ShooterSubsystem(ShooterIO io) {
        instance = this;

        m_io = io;
        m_inputs = new ShooterIOInputsAutoLogged();

        RobotEvent.OnShootingStart.addListener(() -> {
            m_kickerStartupTimer.reset();
            m_kickerStartupTimer.start();
            m_mechanismsPastThreshold = false;
        });
        RobotEvent.OnShootingEnd.addListener(() -> {
            // StateMachine.setIndexerShooterStuff(false);
            m_kickerStartupTimer.stop();
            m_mechanismsPastThreshold = false;
        });
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Shooter", m_inputs);
        Logger.recordOutput("ShooterShouldIndex", shouldIndex());

        if (ZeroingCommand.isSubsystemZeroing(this))
            return;

        if (m_autoStowTimer.isRunning() && m_autoStowTimer.hasElapsed(hoodAutoStowSeconds)) {
            m_autoStowTimer.stop();
            m_autoStowTimer.reset();
            hoodStow();
        }

        final boolean launcherTuning = LiveConfig.getLauncherTuning();
        switch (StateMachine.getShooterState()) {
            case Idle:
                if (launcherTuning) {
                    m_io.hoodAngle(Degrees.of(LiveConfig.getLauncherTuningAngle()));
                    m_io.flywheelVelocity(RPM.of(LiveConfig.getLauncherTuningRPM()));
                    m_io.upperKickerVelocity(upperKickerVelocity);
                } else {
                    m_io.flywheelStop();
                    m_io.upperKickerStop();
                }
                break;
            case Shooting:
                if (!launcherTuning) {
                    if (StateMachine.getLauncherTarget() == LauncherTarget.AllianceFeed) {
                        setParams(StateMachine.isRobotFar() ? allianceFeedParamsFar : allianceFeedParamsNeutralZone);
                    } else {
                        var targetPose = StateMachine.getLauncherTargetPose();
                        InterpolationShooterParams params = null;

                        if (Controls.visionFail1.getAsBoolean())
                            params = paramMap.map.get(3.249d);
                        // if (LiveConfig.getIsPit())
                        else if (LiveConfig.getVisionFail())
                            params = hubCenterParams;
                        else if (targetPose.isPresent()) {
                            // if (StateMachine.getLauncherTargetPoseIsHub())
                            //     paramsOptional = LauncherLocationParam.getFromRobot(StateMachine.odometryAndVision.getEstimatedPose().getTranslation())
                            //         .map((value) -> value.m_params);
                            // else

                            params = calculateParams(targetPose.get());
                        }
                        if (params != null)
                            setParams(params);
                    }
                }
                // if (shouldIndex())
                //     m_io.upperKickerVelocity(upperKickerVelocity);
                m_io.upperKickerVelocity(upperKickerVelocity);
                break;
            case Reversing:
                // m_io.flywheelVelocity(flywheelVelocityReverse);
                m_io.upperKickerVelocity(upperKickerVelocityReverse);
                break;
        }
    }

    public Command zeroMechanisms(boolean skipDrive) {
        return skipDrive
            ? Commands.runOnce(m_io::hoodZero, this)
            : new ZeroingCommand(this, m_io::hoodZeroingDrive, m_io::hoodCanZero, m_io::hoodStop, m_io::hoodZero);
    }

    public Angle getHoodAngle() {
        return Degrees.of(m_inputs.hoodPositionDegrees);
    }

    public void hoodStow() {
        m_io.hoodAngle(hoodPositionHome);
    }

    @AutoLogOutput(key="Shooter/ShouldIndex")
    public boolean shouldIndex() {
        // TODO: if we continue to not use the timer, use a boolean instead of the timer
        if (!m_kickerStartupTimer.isRunning())
            return false;

        final double flywheelTarget = m_inputs.flywheelTargetVelocityRPS;
        final double kickerTarget = m_inputs.upperKickerTargetVelocityRPS;

        if (flywheelTarget == 0d || kickerTarget == 0d)
            return false;

        final boolean flywheel = Math.abs(flywheelTarget - m_inputs.flywheelMotorTopLeftVelocityRPS) <= shouldIndexFlywheelVelocityThresholdRPS;
        final boolean kicker = Math.abs(kickerTarget - m_inputs.upperKickerMotorVelocityRPS) <= shouldIndexKickerVelocityThresholdRPS;

        final boolean indexer = IndexerSubsystem.instance.getIsPastThreshold();

        // // return flywheel && kicker;
        // return flywheel;

        // if (!m_flywheelHasSpunUp && flywheel)
        //     m_flywheelHasSpunUp = true;
        // // if (m_kickerStartupTimer.hasElapsed(shooterReverseCountSeconds))
        // //     StateMachine.setWantsReverseIndexerBeforeShoot(false);
        // if (m_kickerStartupTimer.hasElapsed(shouldIndexKickerVelocityThresholdSeconds))
        //     StateMachine.setIndexerShooterStuff(true);
        // // return m_flywheelHasSpunUp && m_kickerStartupTimer.hasElapsed(shouldIndexKickerVelocityThresholdSeconds);
        // return m_flywheelHasSpunUp;

        if (!m_mechanismsPastThreshold && flywheel && kicker && indexer) {
            m_mechanismsPastThreshold = true;
            StateMachine.indexingProcessStart();
        }
        return m_mechanismsPastThreshold;
    }

    private void setParams(InterpolationShooterParams params) {
        m_io.hoodAngle(Degrees.of(params.degrees()));
        m_io.flywheelVelocity(RPM.of(params.rpm()));
    }

    // private LinearVelocity m_exitVelocity = MetersPerSecond.of(0d);
    // public LinearVelocity getExitVelocity() {
    //     return m_exitVelocity;
    // }
    public LinearVelocity getExitVelocity() {
        final double multiplier = 1.07d;

        // return InchesPerSecond.of(m_inputs.flywheelMotorLeftVelocityRPS * (2d * Math.PI) * flywheelRadius.in(Inches) / 2d);
        final double vb = m_inputs.flywheelMotorTopLeftVelocityRPS * (2d * Math.PI) * flywheelRadiusBig.in(Inches);
        final double vs = m_inputs.flywheelMotorTopLeftVelocityRPS * (2d * Math.PI) * flywheelRadiusSmall.in(Inches);
        return InchesPerSecond.of(multiplier * (vb + vs) / 2d);
    }

    public void stateUpdate(ShooterState shooterState) {
        m_autoStowTimer.stop();
        switch (shooterState) {
            case Idle:
                m_io.upperKickerStop();
            case Reversing:
                m_autoStowTimer.reset();
                m_autoStowTimer.start();
                break;
            case Shooting:
                break;
        }
    }

    // https://github.com/hammerheads5000/2026Rebuilt/blob/6ecae474f5ed81970d8727d2fe6b17e945a1f08f/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java#L100C1-L127C1
    // see 5000-License.md
    public static record PhysicsShotData(LinearVelocity velocity, Angle angle, Translation3d target) {};
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }
    // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    public InterpolationShooterParams calculateParams(Translation3d target) {
        final var robotPose = StateMachine.odometryAndVision.getEstimatedPose();
        final var fieldSpeeds = Drive.instance.getChassisSpeeds();
        
        // https://github.com/hammerheads5000/2026Rebuilt/blob/9a94e647443d8a5651b044449cc5ebb8195efc52/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java#L129
        // see 5000-License.md
        final int iterations = 3;

        double distance = getDistanceToTarget(robotPose, target).in(Meters);
        InterpolationShooterParams shot = paramMap.map.get(distance);
        // shot = new ShotData(shot.exitVelocity, shot.hoodAngle, target);
        Time timeOfFlight = Seconds.of(timeOfFlightMap.get(distance));
        Translation3d predictedTarget = target;

        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            distance = getDistanceToTarget(robotPose, predictedTarget).in(Meters);
            shot = paramMap.map.get(distance);
            // shot = new ShotData(shot.exitVelocity, shot.hoodAngle, predictedTarget);
            timeOfFlight = Seconds.of(timeOfFlightMap.get(distance));
        }

        // return paramMap.map.get(distance);
        return shot;
    }
}
