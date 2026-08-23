package frc.robot.state_machine;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.util.CommandUtil.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.ground_intake.GroundIntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.light_emitting_diodes.LightEmittingDiodesSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.HubActiveState;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class StateMachine {
    public static Alliance ALLIANCE = null;

    public static Distance allianceFlipX(Distance x) {
        return ALLIANCE == Alliance.Red ? FieldConstants.FIELD_LENGTH.minus(x) : x;
    }

    public static Distance allianceFlipY(Distance y) {
        return ALLIANCE == Alliance.Red ? FieldConstants.FIELD_WIDTH.minus(y) : y;
    }

    public static Translation2d allianceFlip(Translation2d translation) {
        return new Translation2d(allianceFlipX(translation.getMeasureX()), allianceFlipY(translation.getMeasureY()));
    }

    public static Rotation2d allianceFlip(Rotation2d rotation) {
        return ALLIANCE == Alliance.Red ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    public static Pose2d allianceFlip(Pose2d pose) {
        return ALLIANCE == Alliance.Red
                ? new Pose2d(allianceFlip(pose.getTranslation()), allianceFlip(pose.getRotation()))
                : pose;
    }

    public static Translation3d allianceFlip(Translation3d translation) {
        return new Translation3d(
                allianceFlipX(translation.getMeasureX()),
                allianceFlipY(translation.getMeasureY()),
                translation.getMeasureZ());
    }

    public static Rotation3d allianceFlip(Rotation3d rotation) {
        return ALLIANCE == Alliance.Red ? rotation.rotateBy(new Rotation3d(0d, 0d, Math.PI)) : rotation;
    }

    public static Pose3d allianceFlip(Pose3d pose) {
        return new Pose3d(allianceFlip(pose.getTranslation()), allianceFlip(pose.getRotation()));
    }

    private static GeneralRobotState generalRobotState = GeneralRobotState.Idle;

    public static GeneralRobotState getGeneralRobotState() {
        return generalRobotState;
    }

    private static Optional<Translation3d> launcherTargetPose = Optional.empty();
    private static boolean launcherTargetPoseIsHub = false;

    private static void setLauncherTargetPose(Optional<Translation3d> pose) {
        if (pose.isPresent() && pose.get() == hubPose) launcherTargetPoseIsHub = true;
        launcherTargetPose = pose;
    }

    public static Optional<Translation3d> getLauncherTargetPose() {
        return launcherTargetPose;
    }

    public static boolean getLauncherTargetPoseIsHub() {
        return launcherTargetPoseIsHub;
    }

    private static IntakeState intakeState = IntakeState.HomeOff;

    public static IntakeState getIntakeState() {
        return intakeState;
    }

    public static void setIntakeState(IntakeState newIntakeState) {
        var old = intakeState;
        intakeState = newIntakeState;
        GroundIntakeSubsystem.instance.stateUpdate(intakeState, old);
        // IndexerSubsystem.instance.intakeStateUpdate(intakeState);
    }

    private static LauncherTarget launcherTarget = LauncherTarget.Idle;

    public static LauncherTarget getLauncherTarget() {
        return launcherTarget;
    }
    // NOTE: Turret stateUpdate requires this to only be called when changed
    public static void setLauncherTarget(LauncherTarget newTarget) {
        launcherTarget = newTarget;
        if (Constants.USE_TURRET) TurretSubsystem.instance.stateUpdate(launcherTarget);

        DriveCommands.resetAngleController();
    }

    private static boolean withinJoystickRotationErrorThreshold = false;

    public static boolean getWithinJoystickRotationErrorThreshold() {
        return withinJoystickRotationErrorThreshold;
    }

    public static void setWithinJoystickRotationErrorThreshold(boolean within) {
        withinJoystickRotationErrorThreshold = within;
    }

    // private static boolean indexerShooterStuff = false;
    // public static boolean getIndexerShooterStuff() {
    //     return indexerShooterStuff;
    // }
    // public static void setIndexerShooterStuff(boolean wants) {
    //     indexerShooterStuff = wants;
    // }

    // private static boolean indexerQuickReverse = false;
    // public static boolean getIndexerQuickReverse() {
    //     return indexerQuickReverse;
    // }
    // public static void setIndexerQuickReverse(boolean wants) {
    //     indexerQuickReverse = wants;
    // }

    private static boolean turretShouldIndexFlagCleared = false;

    public static boolean turretShouldIndex() {
        if (LiveConfig.getIsPit()) return true;
        final boolean allianceFeedGood =
                getLauncherTarget() == LauncherTarget.AllianceFeed ? isFacingAllianceZone() : false;
        return Constants.USE_TURRET
                ? TurretSubsystem.instance.shouldIndex()
                : Controls.lockWheelsButton.getAsBoolean()
                        || (withinJoystickRotationErrorThreshold || allianceFeedGood);
    }

    public static boolean shouldIndex() {
        boolean turretShouldIndex = turretShouldIndex();
        if (turretShouldIndex) turretShouldIndexFlagCleared = true;

        if (getLauncherTarget() != LauncherTarget.AllianceFeed) turretShouldIndex = turretShouldIndexFlagCleared;

        return (turretShouldIndex || Controls.forceTurretShouldIndex.getAsBoolean())
                && ShooterSubsystem.instance.shouldIndex();
    }

    private static ShooterState shooterState = ShooterState.Idle;

    public static ShooterState getShooterState() {
        return shooterState;
    }

    public static void setShooterState(ShooterState newShooterState) {
        final var oldShooterState = shooterState;
        shooterState = newShooterState;
        ShooterSubsystem.instance.stateUpdate(newShooterState);
        if (newShooterState == ShooterState.Shooting) RobotEvent.OnShootingStart.trigger();
        else if (oldShooterState != ShooterState.Shooting) {
            RobotEvent.OnShootingEnd.trigger();
            turretShouldIndexFlagCleared = false;
        }
    }

    private static ClimberState climberState = ClimberState.Idle;

    public static ClimberState getClimberState() {
        return climberState;
    }

    public static void setClimberState(ClimberState newClimberState) {
        climberState = newClimberState;
        ClimberSubsystem.instance.stateUpdate(climberState);
    }

    private static final Timer m_debugTimer1 = new Timer();

    public static final class RobotCommands {
        // intake
        public static Command setIntakeState(IntakeState intakeState) {
            return GroundIntakeSubsystem.instance.runOnce(() -> StateMachine.setIntakeState(intakeState));
        }

        public static Command deployIntakeOff() {
            return cmdName(setIntakeState(IntakeState.DeployedOff), "IntakeDeployedOff");
        }

        public static Command deployIntakeOn() {
            return cmdName(setIntakeState(IntakeState.DeployedOn), "IntakeDeployedOn");
        }

        public static Command retractIntakeOff() {
            return cmdName(setIntakeState(IntakeState.HomeOff), "IntakeHomeOff");
        }

        public static Command oscillateIntakeOff() {
            return cmdName(setIntakeState(IntakeState.OscillateOff), "IntakeOscillatedOff");
        }

        public static Command oscillateIntakeOn() {
            return cmdName(setIntakeState(IntakeState.OscillateOn), "IntakeOscillatedOn");
        }

        public static Command intakeDynamicOn() {
            return cmdName(
                    Commands.defer(() -> setIntakeState(intakeState.on()), Set.of(GroundIntakeSubsystem.instance)),
                    "DynamicOn");
        }

        public static Command intakeDynamicOff() {
            return cmdName(
                    Commands.defer(() -> setIntakeState(intakeState.off()), Set.of(GroundIntakeSubsystem.instance)),
                    "DynamicOff");
        }

        public static Command intakeDynamicReverse() {
            return cmdName(
                    Commands.defer(() -> setIntakeState(intakeState.reverse()), Set.of(GroundIntakeSubsystem.instance)),
                    "DynamicReverse");
        }

        public static Command intakeDeployedToOnConditional() {
            return cmdName(
                    Commands.either(deployIntakeOn(), Commands.none(), () -> intakeState.isDeployed()),
                    "IntakeDeployedToOnConditional");
        }

        public static Command intakeDeployedToOffConditional() {
            return cmdName(
                    Commands.either(deployIntakeOff(), Commands.none(), () -> intakeState.isDeployed()),
                    "IntakeDeployedToOffConditional");
        }

        public static Command intakeOscillateToOnConditional() {
            return cmdName(
                    Commands.either(oscillateIntakeOn(), Commands.none(), () -> intakeState.isOscillate()),
                    "IntakeOscillateToOnConditional");
        }

        public static Command intakeOscillateToOffConditional() {
            return cmdName(
                    Commands.either(oscillateIntakeOff(), Commands.none(), () -> intakeState.isOscillate()),
                    "IntakeOscillateToOffConditional");
        }

        // turret
        public static Command setLauncherTarget(LauncherTarget turretState) {
            Command command = Commands.runOnce(() -> StateMachine.setLauncherTarget(turretState));
            if (Constants.USE_TURRET) command.addRequirements(TurretSubsystem.instance);
            return command;
        }

        public static Command setLauncherTargetIdle() {
            return cmdName(setLauncherTarget(LauncherTarget.Idle), "LauncherTargetIdle");
        }

        public static Command setLauncherTargetHubTracking() {
            return cmdName(setLauncherTarget(LauncherTarget.HubTracking), "LauncherTargetHubTracking");
        }

        public static Command setLauncherTargetAllianceFeed() {
            return cmdName(setLauncherTarget(LauncherTarget.AllianceFeed), "LauncherTargetAllianceFeed");
        }

        // shooter
        public static Command setShooterState(ShooterState shooterState) {
            return ShooterSubsystem.instance.runOnce(() -> StateMachine.setShooterState(shooterState));
        }

        public static Command setShooterIdle() {
            return cmdName(setShooterState(ShooterState.Idle), "ShooterIdle");
        }

        public static Command setShooterShooting() {
            return cmdName(setShooterState(ShooterState.Shooting), "ShooterShooting");
        }

        public static Command setShooterReversing() {
            return cmdName(setShooterState(ShooterState.Reversing), "ShooterReversing");
        }

        // shooter + turret
        public static Command engageShooterHub() {
            // return cmdName(setLauncherTargetHubTracking()
            return cmdName(
                    Commands.runOnce(() -> {
                                m_debugTimer1.restart();
                            })
                            .andThen(setLauncherTargetHubTracking())
                            // .andThen(Commands.waitUntil(StateMachine::turretShouldIndex))
                            .andThen(setShooterShooting()),
                    "EngageShooterHub");
        }

        public static Command engageShooterAllianceFeed() {
            return cmdName(
                    setLauncherTargetAllianceFeed()
                            // .andThen(Commands.waitUntil(StateMachine::turretShouldIndex))
                            .andThen(setShooterShooting()),
                    "EngageShooterAllianceFeed");
        }

        public static Command disengageShooter() {
            return cmdName(setLauncherTargetIdle().alongWith(setShooterIdle()), "DisengageShooter");
        }

        // climber
        public static Command setClimberState(ClimberState climberState) {
            return ClimberSubsystem.instance.runOnce(() -> {
                StateMachine.setClimberState(climberState);
            });
        }

        public static Command climberHome() {
            return cmdName(setClimberState(ClimberState.Home), "ClimberHome");
        }

        public static Command climberDeployed() {
            return cmdName(setClimberState(ClimberState.Deployed), "ClimberDeployed");
        }

        // general
        public static Command generalReversing() {
            return cmdName(setShooterReversing().alongWith(intakeDynamicReverse()), "GeneralReversing");
        }
        // idle those that reversing affects
        public static Command generalReversingIdle() {
            return cmdName(setShooterIdle().alongWith(intakeDynamicOff()), "GeneralReversingIdle");
        }

        public static Command zeroMechanisms(boolean skipDrive, InterruptionBehavior interruptionBehavior) {
            return cmdName(
                    Commands.parallel(
                                    ShooterSubsystem.instance.zeroMechanisms(skipDrive),
                                    GroundIntakeSubsystem.instance.zeroMechanisms(skipDrive)
                                    // GroundIntakeSubsystem.instance.zeroMechanisms(true)
                                    )
                            .withInterruptBehavior(interruptionBehavior)
                            .ignoringDisable(true),
                    "ZeroMechanisms");
        }
    }

    public static Rotation2d initialSwerveRotation = null;

    public static Angle getRobotRotationalGoal(Translation3d targetPose) {
        Pose2d robotPose = StateMachine.odometryAndVision.getEstimatedPose();
        final ChassisSpeeds robotSpeeds = Drive.instance.getChassisSpeeds();

        // t1
        // robotPose = robotPose.exp(new Twist2d(
        //     robotSpeeds.vxMetersPerSecond * lookAheadTime,
        //     robotSpeeds.vyMetersPerSecond * lookAheadTime,
        //     robotSpeeds.omegaRadiansPerSecond * lookAheadTime
        // ));

        // t0
        // Translation2d robotTranslation = robotPose.getTranslation();
        // Translation2d targetTranslation = targetPose.getTranslation().toTranslation2d();

        // Rotation2d targetRotation = targetTranslation.minus(robotTranslation).getAngle();
        // Rotation2d currentRotation = robotPose.getRotation();

        // Rotation2d rotationError = targetRotation.minus(currentRotation);

        // final double targetangle = rotationError.getRadians();

        // return Radians.of(targetangle);

        final Translation2d robotTranslation = robotPose.getTranslation();
        final Translation2d targetTranslation = targetPose.toTranslation2d();

        Translation2d yawTranslation = targetTranslation.minus(robotTranslation);
        // Translation2d yawTranslation = robotTranslation.minus(targetTranslation);
        yawTranslation =
                yawTranslation.minus(new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond));

        final Rotation2d currentRotation = robotPose.getRotation();
        final Rotation2d rotationError = yawTranslation.getAngle().minus(currentRotation);

        final double targetangle = rotationError.getRadians();

        return Radians.of(targetangle);
    }

    private static final Translation3d hubPoseBlue =
            new Translation3d(Inches.of(182.11d), Inches.of(158.84d), Feet.of(6d));
    private static Translation3d hubPose = new Translation3d();

    public static Translation3d getHubPose() {
        return hubPose;
    }

    public static Translation3d getHubPoseBlue() {
        return hubPoseBlue;
    }

    // https://github.com/hammerheads5000/2026Rebuilt/blob/b32c384e337b1a89d3d651c05696c9366a105504/src/main/java/frc/robot/Constants.java#L398
    // see 5000-License.md
    // public static final Translation3d allianceFeedLeft = new Translation3d(
    //     Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).plus(Inches.of(85)), Inches.zero());
    // public static final Translation3d allianceFeedCenter =
    //     new Translation3d(Inches.of(90), FieldConstants.FIELD_WIDTH.div(2), Inches.zero());
    // public static final Translation3d allianceFeedRight = new Translation3d(
    //     Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).minus(Inches.of(85)), Inches.zero());

    // public static final Translation3d getAllianceFeedPose() {
    //     return allianceFlip(isRobotOnAllianceLeft() ? allianceFeedLeft : allianceFeedRight);
    // }

    public static final OdometryAndVision odometryAndVision = new OdometryAndVision();

    private static boolean isRobotOnAllianceLeft() {
        final var y = odometryAndVision.getEstimatedPose().getMeasureY();
        final var threshold = FieldConstants.FIELD_WIDTH.div(2d);

        if (ALLIANCE == Alliance.Red) return y.lte(threshold);
        else return y.gt(threshold);
    }

    public static boolean isRobotFar() {
        final var x = odometryAndVision.getEstimatedPose().getMeasureX();

        final double redNumerator = 1d;
        final double denominator = 4d;

        if (ALLIANCE == Alliance.Red) return x.lte(FieldConstants.FIELD_LENGTH.times(redNumerator / denominator));
        else return x.gte(FieldConstants.FIELD_LENGTH.times((denominator - redNumerator) / denominator));
    }

    public static boolean isFacingAllianceZone() {
        final var targetRotation = ALLIANCE == Alliance.Red ? Rotation2d.kZero : Rotation2d.k180deg;
        final var offset = odometryAndVision.getRotation().minus(targetRotation).getDegrees();
        return Math.abs(offset) <= DriveConstants.IS_FACING_ALLIANCE_ZONE_THRESHOLD;
    }

    public static boolean wantsToShoot() {
        return shooterState == ShooterState.Shooting;
    }

    public static boolean isShooting() {
        return generalRobotState.isFiring();
    }

    private static final Timer turboTimer = new Timer();
    private static final double TURBO_MAX = 15d; // seconds

    public static void startTurbo() {
        turboTimer.start();
    }

    public static void stopTurbo() {
        turboTimer.stop();
    }

    public static boolean isTurboAvailable() {
        return !turboTimer.hasElapsed(TURBO_MAX);
    }

    static {
        RobotEvent.TurboOn.addListener(StateMachine::startTurbo);
        RobotEvent.TurboOff.addListener(StateMachine::stopTurbo);
    }

    public static Command eventTurboOff() {
        return Commands.either(Commands.runOnce(RobotEvent.TurboOff::trigger), Commands.none(), turboTimer::isRunning);
    }

    static {
        SmartDashboard.putNumber("DebugTimer1", -1d);
    }

    public static void indexingProcessStart() {
        m_debugTimer1.stop();
        SmartDashboard.putNumber("DebugTimer1", m_debugTimer1.get());
    }

    private static double hubDistanceMeters = 4d;

    public static double getHubDistanceMeters() {
        return hubDistanceMeters;
    }

    private static boolean needsToUpdateRobot = true;

    @SuppressWarnings("unused") // for useTurret
    public static synchronized void periodic(Robot robot) {
        Logger.recordOutput("Odometry/Robot", odometryAndVision.getEstimatedPose());

        Logger.recordOutput("StateMachine/IntakeState", intakeState);
        Logger.recordOutput("StateMachine/LauncherTarget", launcherTarget);
        Logger.recordOutput("StateMachine/ShooterState", shooterState);
        Logger.recordOutput("StateMachine/ClimberState", climberState);

        Logger.recordOutput("StateMachine/TurboMeter", turboTimer.get());

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() != ALLIANCE) {
            needsToUpdateRobot = true;

            ALLIANCE = alliance.get();
            Logger.recordOutput("StateMachine/Alliance", ALLIANCE);

            // AprilTagConstants.update(ALLIANCE);

            if (ALLIANCE == Alliance.Blue) initialSwerveRotation = Rotation2d.kZero;
            else initialSwerveRotation = Rotation2d.kPi;

            // VisionSubsystem.update();
            // for (var value : VisionSubsystem.RelativeReefLocation.values())
            //     value.update();
            // for (var value : VisionSubsystem.CoralStationID.values())
            //     value.update();

            hubPose = allianceFlip(hubPoseBlue);
        }

        if (needsToUpdateRobot && robot != null && ALLIANCE != null) {
            needsToUpdateRobot = false;

            // if (m_visionSubsystem != null && ALLIANCE != null)
            //     VisionSubsystem.RelativeReefLocation.postUpdate(m_visionSubsystem);
        }

        if (Constants.USE_TURRET ? TurretSubsystem.instance.getManualEnabled() : false)
            setLauncherTargetPose(Optional.empty());
        else {
            setLauncherTargetPose(
                    switch (StateMachine.getLauncherTarget()) {
                        case HubTracking -> Optional.of(hubPose);
                            // case AllianceFeed -> Optional.of(getAllianceFeedPose());
                        default -> Optional.empty();
                    });
        }

        // set generalRobotState
        {
            final boolean isFiring = IndexerSubsystem.instance.getIsFeeding();

            switch (launcherTarget) {
                case HubTracking:
                    generalRobotState = isFiring ? GeneralRobotState.HubTrackingFiring : GeneralRobotState.HubTracking;
                    break;
                case AllianceFeed:
                    generalRobotState =
                            isFiring ? GeneralRobotState.AllianceFeedFiring : GeneralRobotState.AllianceFeed;
                    break;
                default:
                    generalRobotState = GeneralRobotState.Idle;
                    break;
            }

            LightEmittingDiodesSubsystem.instance.stateUpdate(generalRobotState);

            Logger.recordOutput("StateMachine/GeneralRobotState", generalRobotState);
        }

        hubDistanceMeters = ShooterSubsystem.getDistanceToTarget(odometryAndVision.getEstimatedPose(), getHubPose())
                .in(Meters);
        Logger.recordOutput("Shooter/HubDistanceMeters", hubDistanceMeters);

        Logger.recordOutput(
                "StateMachine/LauncherTargetPose",
                new Pose3d(launcherTargetPose.orElse(new Translation3d()), new Rotation3d()));

        Logger.recordOutput("StateMachine/ShouldIndex", shouldIndex());
        Logger.recordOutput("StateMachine/TurretShouldIndex", turretShouldIndex());
        Logger.recordOutput("StateMachine/TurretShouldIndexFlag", turretShouldIndexFlagCleared);
        Logger.recordOutput("StateMachine/WithinJoystickRotationErrorThreshold", withinJoystickRotationErrorThreshold);
        Logger.recordOutput("StateMachine/IsRobotOnAllianceLeft", isRobotOnAllianceLeft());
        Logger.recordOutput("StateMachine/IsRobotFar", isRobotFar());
        Logger.recordOutput("StateMachine/IsRobotFacingAllianceZone", isFacingAllianceZone());

        HubActiveState.periodic();

        Logger.recordOutput("StateMachine/HubActive", HubActiveState.isOurHubActive());
        Logger.recordOutput("StateMachine/TimeUntilSwap", HubActiveState.timeUntilSwap());
    }
}
