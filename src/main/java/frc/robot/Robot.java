// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.hoodPositionHome;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;
import static frc.robot.util.CommandUtil.*;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.SimulationCommands;
import frc.robot.commands.SimulationCommands.SimFuelCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.state_machine.ClimberState;
import frc.robot.state_machine.IntakeState;
import frc.robot.state_machine.LiveConfig;
import frc.robot.state_machine.RobotEvent;
import frc.robot.state_machine.StateMachine;
import frc.robot.state_machine.StateMachine.RobotCommands;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.ground_intake.GroundIntakeIO;
import frc.robot.subsystems.ground_intake.GroundIntakeIOReal;
import frc.robot.subsystems.ground_intake.GroundIntakeIOSim;
import frc.robot.subsystems.ground_intake.GroundIntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.light_emitting_diodes.LightEmittingDiodesIO;
import frc.robot.subsystems.light_emitting_diodes.LightEmittingDiodesSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOMackinac;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.ApriltagUtil;
import frc.robot.util.ConversionUtil;
import frc.robot.util.SystemTimeValidReader;
import frc.robot.util.PhoenixUtil.MotorAction;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private CommandScheduler m_commandScheduler = null;
    private Command m_autoCommand = null;

    // subsystems
    private Drive m_driveSubsystem = null;
    private TurretSubsystem m_turretSubsystem = null;
    @SuppressWarnings("unused")
    private GroundIntakeSubsystem m_intakeSubsystem = null;
    private IndexerSubsystem m_indexerSubsystem = null;
    private ShooterSubsystem m_shooterSubsystem = null;
    @SuppressWarnings("unused")
    private ClimberSubsystem m_climberSubsystem = null;
    @SuppressWarnings("unused")
    private VisionSubsystem m_visionSubsystem = null;
    @SuppressWarnings("unused")
    private LightEmittingDiodesSubsystem m_lightEmittingDiodesSubsystem = null;

    private final LoggedDashboardChooser<Command> m_autoChooser;

    @AutoLogOutput(key="RotationTargetOffset")
    private double m_rotationTargetMeterOffset = 0d;

    private static final double rotationTargetMax = Units.inchesToMeters(24d);
    private static final double rotationTargetMin = -rotationTargetMax;

    public Robot() {
        // super(0.01);
        SignalLogger.setPath("/U/logs");
        SignalLogger.start();

        // RobotController.setBrownoutVoltage(6d); // :DDDDD
        
        m_commandScheduler = CommandScheduler.getInstance();

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
              "GitDirty",
              switch (BuildConstants.DIRTY) {
                case 0 -> "All changes committed";
                case 1 -> "Uncommitted changes";
                default -> "Unknown";
              });

        // Set up data receivers & replay source
        switch (Constants.CURRENT_MODE) {
            case REAL: {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());

                m_driveSubsystem = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                if (Constants.USE_TURRET)
                    m_turretSubsystem = new TurretSubsystem(new TurretIOReal());
                m_intakeSubsystem = new GroundIntakeSubsystem(new GroundIntakeIOReal());
                m_indexerSubsystem = new IndexerSubsystem(new IndexerIOReal());
                m_shooterSubsystem = new ShooterSubsystem(new ShooterIOReal());
                m_climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
                // m_lightEmittingDiodesSubsystem = new LightEmittingDiodesSubsystem(new LightEmittingDiodesIOReal());
                m_lightEmittingDiodesSubsystem = new LightEmittingDiodesSubsystem(new LightEmittingDiodesIO() {});

                break;
            }
            case SIM: {
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());

                m_driveSubsystem = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                if (Constants.USE_TURRET)
                    m_turretSubsystem = new TurretSubsystem(new TurretIOSim());
                m_intakeSubsystem = new GroundIntakeSubsystem(new GroundIntakeIOSim());
                m_indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
                m_shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
                m_climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
                m_lightEmittingDiodesSubsystem = new LightEmittingDiodesSubsystem(new LightEmittingDiodesIO() {});
                break;
            }
            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

                m_driveSubsystem = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                if (Constants.USE_TURRET)
                    m_turretSubsystem = new TurretSubsystem(new TurretIO() {});
                m_intakeSubsystem = new GroundIntakeSubsystem(new GroundIntakeIO() {});
                m_indexerSubsystem = new IndexerSubsystem(new IndexerIO() {});
                m_shooterSubsystem = new ShooterSubsystem(new ShooterIO() {});
                m_climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
                m_lightEmittingDiodesSubsystem = new LightEmittingDiodesSubsystem(new LightEmittingDiodesIO() {});
                break;
        }

        // TODO: vision sim?
        if (Constants.CURRENT_MODE == Mode.REAL || Constants.CURRENT_MODE == Mode.SIM) {
            if (Constants.USE_MACKINAC)
                m_visionSubsystem = new VisionSubsystem(
                    ApriltagUtil.fieldLayout,
                    new VisionIOMackinac(ApriltagUtil.fieldLayout, 0),
                    new VisionIOMackinac(ApriltagUtil.fieldLayout, 1),
                    new VisionIOMackinac(ApriltagUtil.fieldLayout, 2),
                    new VisionIOMackinac(ApriltagUtil.fieldLayout, 3)
                );
            else
                m_visionSubsystem = new VisionSubsystem(new VisionIOLimelight());
        } else {
            if (Constants.USE_MACKINAC)
                m_visionSubsystem = new VisionSubsystem(
                    ApriltagUtil.fieldLayout,
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {});
            else
                m_visionSubsystem = new VisionSubsystem(new VisionIO() {});
        }

        // Start AdvantageKit logger
        Logger.start();

        DriverStation.silenceJoystickConnectionWarning(true);
        m_commandScheduler.setPeriod(0.04d);

        SystemTimeValidReader.start();

        StateMachine.periodic(this);
        setupNamedCommands();
        m_driveSubsystem.configurePathPlanner();
        
        m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        m_autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_driveSubsystem));
        m_autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_driveSubsystem));

        SmartDashboard.putBoolean("NTGyro", false);
        resetGyro(Rotation2d.kZero);
        configureButtons();
        m_commandScheduler.onCommandInterrupt((Command command, Optional<Command> interruptor) -> {
            System.out.println(command.getName() + " was interrupted by " + (interruptor.isPresent() ? interruptor.get().getName() : "null"));
        });
    }

    private void configureButtons() {
        final DoubleSupplier driverX = () -> -Controls.driveController.getLeftY();
        final DoubleSupplier driverY = () -> -Controls.driveController.getLeftX();
        final DoubleSupplier driverOmega = () -> -Controls.driveController.getRightX();

        m_driveSubsystem.setDefaultCommand(
            cmdName(DriveCommands.joystickDrive(
                m_driveSubsystem,
                driverX,
                driverY,
                driverOmega,
                getShooterRotationalGoalSupplier()), "JoystickDrive"));
        
        Controls.resetGyroButton.onTrue(cmdName(Commands.runOnce(this::resetGyro), "ResetGyro"));

        Controls.lockWheelsButton.whileTrue(
            cmdName(Commands.run(m_driveSubsystem::stopWithX, m_driveSubsystem), "LockWheels"));

        // when deployIntakeButton pressed, already deployed / has been 0.25 seconds -> deploy + spin rollers OTHERWISE deploy + don't spin rollers
        Controls.deployIntakeButton
            .onTrue(cmdName(Commands.either(
                RobotCommands.deployIntakeOn(),
                RobotCommands.deployIntakeOff(),
                () -> StateMachine.getIntakeState().isDeployed()
            ), "DeployIntakeInitial"));
        Controls.deployIntakeButton
            .debounce(0.25d)
            .onTrue(cmdName(RobotCommands.intakeDeployedToOnConditional(), "DeployIntakeDelayed"));
        Controls.deployIntakeButton.onFalse(cmdName(RobotCommands.intakeDeployedToOffConditional(), "DeployIntakeOff"));

        // Controls.deployIntakeButton.debounce(0.5d).whileTrue(
        //     IntakeCommands.joystickAssist(m_driveSubsystem, driverX, driverY, driverOmega, () -> false));

        Controls.retractIntakeButton.onTrue(RobotCommands.retractIntakeOff());

        Controls.reverseButton.onTrue(RobotCommands.generalReversing());
        Controls.reverseButton.onFalse(RobotCommands.generalReversingIdle());

        Controls.hubButton.onTrue(
            RobotCommands.engageShooterHub()
        );
        Controls.hubButton.onFalse(
            cmdName(Commands.either(
                RobotCommands.engageShooterAllianceFeed(),
                RobotCommands.disengageShooter(),
                Controls.allianceFeedButton::getAsBoolean), "HubDisengage")
        );

        Controls.allianceFeedButton.onTrue(
            RobotCommands.engageShooterAllianceFeed()
        );
        Controls.allianceFeedButton.onFalse(
            cmdName(Commands.either(
                RobotCommands.engageShooterHub(),
                RobotCommands.disengageShooter(),
                Controls.hubButton::getAsBoolean), "AllianceFeedDisengage")
        );

        // Controls.allianceFeedNoTurretButton.onTrue(
        //     RobotCommands.setShooterShooting()
        // );
        // Controls.allianceFeedNoTurretButton.onFalse(
        //     RobotCommands.setShooterIdle()
        // );

        final double rotationTargetOffsetAddend = Units.inchesToMeters(3d);
        Controls.rotationTargetBumpLeft
            .and(Controls.rotationTargetBumpRight.negate())
            .onTrue(cmdName(Commands.runOnce(() -> {
                m_rotationTargetMeterOffset -= rotationTargetOffsetAddend;
                if (m_rotationTargetMeterOffset < rotationTargetMin)
                    m_rotationTargetMeterOffset = rotationTargetMin;
            }), "RotationTargetBumpLeft"));
        Controls.rotationTargetBumpRight
            .and(Controls.rotationTargetBumpLeft.negate())
            .onTrue(cmdName(Commands.runOnce(() -> {
                m_rotationTargetMeterOffset += rotationTargetOffsetAddend;
                if (m_rotationTargetMeterOffset > rotationTargetMax)
                    m_rotationTargetMeterOffset = rotationTargetMax;
            }), "RotationTargetBumpRight"));

        Controls.rotationTargetBumpLeft
            .and(Controls.rotationTargetBumpRight)
            .onTrue(cmdName(Commands.runOnce(() -> m_rotationTargetMeterOffset = 0d), "RotationTargetBumpReset"));

        // NOTE: turbo mode is used in ModuleIOTalonFX.java
        // Controls.turboButton
        //     .whileTrue(Commands.either(
        //         Commands.runOnce(RobotEvent.TurboOn::trigger)
        //             .andThen(Commands.waitUntil(() -> !StateMachine.isTurboAvailable()))
        //             .andThen(StateMachine.eventTurboOff()),
        //         Commands.none(),
        //         StateMachine::isTurboAvailable))
        //     .onFalse(StateMachine.eventTurboOff());

        if (Robot.isSimulation())
            new Trigger(() -> StateMachine.isShooting())
                .whileTrue(repeatingSimFuelCommand());

        // climber
        Controls.climbUp.onTrue(RobotCommands.setIntakeState(IntakeState.OscillateOff)
            .andThen(Commands.waitSeconds(0.5d))
            .andThen(RobotCommands.climberDeployed()));
        Controls.climbDown.onTrue(RobotCommands.climberHome());

        if (Constants.USE_TURRET) {
            Controls.manualTurretToggle.onTrue(m_turretSubsystem.toggleManualModeCommand());

            m_turretSubsystem.setManualSupplier(() -> {
                var controller = Controls.operatorController;
                if (Math.abs(controller.getRawAxis(XboxController.Axis.kRightX.value)) >= 0.15)
                    return controller.getRightX() * 2.5;
                return 0d;
            });
        }
    }

    private boolean m_hasLoweredDriveStator = false;
    private void lowerDriveStator() {
        if (!m_hasLoweredDriveStator) {
            m_hasLoweredDriveStator = true;
            m_driveSubsystem.lowerDriveCurrentLimits();
        }
    }

    private void setupNamedCommands() {
        var engageShooterHub = cmdName(RobotCommands.engageShooterHub()
            .andThen(Commands.runOnce(() -> {
                lowerDriveStator();
            })), "Auto_EngageShooterHub");
        NamedCommands.registerCommand("EngageShooterHub", engageShooterHub);
        NamedCommands.registerCommand("DisengageShooter", RobotCommands.disengageShooter());

        NamedCommands.registerCommand("IntakeDeployOn", RobotCommands.deployIntakeOn());
        NamedCommands.registerCommand("IntakeHomeOff", RobotCommands.retractIntakeOff());

        // FIXME: waitForEmptyHopper command using vision
        Command waitForEmptyHopper = Commands.waitSeconds(5d); // 12
        // NamedCommands.registerCommand("AfterShoot", cmdName(waitForEmptyHopper
        //     .andThen(
        //         RobotCommands.disengageShooter())
        //     .andThen(
        //         RobotCommands.setShooterReversing()
        //             .andThen(Commands.waitSeconds(0.4d))
        //             .andThen(RobotCommands.setShooterIdle())
        //     ), "AfterShoot"));
        NamedCommands.registerCommand("AfterShoot", cmdName(waitForEmptyHopper.andThen(RobotCommands.disengageShooter()), "AfterShoot"));

        // NamedCommands.registerCommand("ClimbRotate", Commands.defer(() -> {
        //     var robotPose = StateMachine.odometryAndVision.getEstimatedPose();
        //     robotPose = new Pose2d(robotPose.getTranslation(), Rotation2d.kZero);
        //     return AutoBuilder.pathfindToPose(robotPose, new PathConstraints(
        //         MetersPerSecond.of(1d), MetersPerSecondPerSecond.of(1d),
        //         RadiansPerSecond.of(360d), RadiansPerSecondPerSecond.of(720d)));
        // }, Set.of()));
        NamedCommands.registerCommand("ClimbRotate", cmdName(AutoDriveCommands.faceRotation2d(StateMachine.allianceFlip(Rotation2d.kZero)), "AutoClimbRotate"));

        NamedCommands.registerCommand("ClimbMark1Deploy", RobotCommands.setClimberState(ClimberState.Deployed));
        NamedCommands.registerCommand("ClimbMark1Home", RobotCommands.setClimberState(ClimberState.Home));
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);

        m_commandScheduler.run();

        Threads.setCurrentThreadPriority(true, 10);

        StateMachine.periodic(this);
        MotorAction.process();
        Logger.recordOutput("Pose3dZero", Pose3d.kZero);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        // resetState();
        RobotEvent.OnDisabled.trigger();
    }

    private void resetState() {
        m_commandScheduler.schedule(RobotCommands.disengageShooter());
        // RobotCommands.setIntakeState(StateMachine.getIntakeState().off());
    }

    private boolean m_lastNTGyroButton = false;
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        final boolean NTGyroButton = SmartDashboard.getBoolean("NTGyro", m_lastNTGyroButton);
        if (NTGyroButton != m_lastNTGyroButton) {
            var command = m_autoChooser.get();

            resetPoseFromPathPlanner(command);
        }
        m_lastNTGyroButton = NTGyroButton;
    }

    private void resetPoseFromPathPlanner(Command command) {
        if (command == null)
            return;

        if (command instanceof PathPlannerAuto) {
            var auto = (PathPlannerAuto) command;
            // resetGyro(auto.getStartingPose().getRotation());
            resetPose(auto.getStartingPose());
        }
    }

    private Supplier<Optional<Rotation2d>> m_autonomousRotationSupplier;
    private Command m_autoSimFuelCommand;

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_hasLoweredDriveStator = false;

        m_driveSubsystem.increaseDriveCurrentLimits();

        // ensure flywheel is spinning if we want it to be
        StateMachine.setShooterState(StateMachine.getShooterState());

        if (m_autonomousRotationSupplier == null)
            m_autonomousRotationSupplier = getShooterRotationalGoalSupplier();

        m_autoCommand = m_autoChooser.get();

        if (m_autoCommand != null)
            m_commandScheduler.schedule(m_autoCommand);
        resetPoseFromPathPlanner(m_autoCommand);

        if (m_autoSimFuelCommand == null)
            m_autoSimFuelCommand = repeatingSimFuelCommand();

        m_commandScheduler.schedule(m_autoSimFuelCommand);

        RobotEvent.OnAutoEnabled.trigger();
        RobotEvent.OnEnabled.trigger();
    }

    /** This function is called periodically during autonomous. */
    private double m_pathplannerFeedbackTimestamp = MathSharedStore.getTimestamp();

    @Override
    public void autonomousPeriodic() {
        final double timestamp = MathSharedStore.getTimestamp();

        var rotation = m_autonomousRotationSupplier.get();
        // TODO: set overrideRotationFeedback once when rotation is present and inside it it will get a new rotation and if it's empty then clear?
        if (rotation.isPresent()) {
            final double output = DriveCommands.calculateOmega(m_driveSubsystem, rotation.get()/*.plus(Rotation2d.kCW_90deg)*/);
            PPHolonomicDriveController.overrideRotationFeedback(() -> {
                m_pathplannerFeedbackTimestamp = timestamp;
                return output;
            });
        } else
            PPHolonomicDriveController.clearRotationFeedbackOverride();

        if (rotation.isPresent() && (timestamp - m_pathplannerFeedbackTimestamp) > 0.05d) {
            final double output = DriveCommands.calculateOmega(m_driveSubsystem, rotation.get()) * m_driveSubsystem.getMaxAngularSpeedRadPerSec();
            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    0d,
                    0d,
                    output);
            boolean isFlipped =
                StateMachine.ALLIANCE == Alliance.Red;
            m_driveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    isFlipped
                    ? StateMachine.odometryAndVision.getRotation().plus(new Rotation2d(Math.PI))
                    : StateMachine.odometryAndVision.getRotation()));
        }
    }
    
    @Override
    public void autonomousExit() {
        StateMachine.setClimberState(ClimberState.Idle);
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        if (m_autoCommand != null) {
            m_commandScheduler.cancel(m_autoCommand);
            m_autoCommand = null;
        }

        m_commandScheduler.cancelAll();

        resetState();
        lowerDriveStator();

        RobotEvent.OnTeleopEnabled.trigger();
        RobotEvent.OnEnabled.trigger();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        m_commandScheduler.cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        SimFuelCommand.step();
    }

    
    private Supplier<Optional<Rotation2d>> getShooterRotationalGoalSupplier() {
        return Constants.USE_TURRET ? () -> Optional.empty() : () -> {
            if (LiveConfig.getIsPit())
                return Optional.empty();

            var targetPose = StateMachine.getLauncherTargetPose();
            if (targetPose.isEmpty())
                return Optional.empty();

            var rotation = StateMachine.odometryAndVision.getRotation();
            var target = targetPose.get();
            target = target.plus(new Translation3d(0d, (StateMachine.ALLIANCE == Alliance.Red ? 1d : -1d) * m_rotationTargetMeterOffset, 0d));
            return Optional.of(rotation.plus(new Rotation2d(StateMachine.getRobotRotationalGoal(target))));
        };
    }

    public void resetGyro(Rotation2d offset) {
        System.out.println("resetGyro called");
        if (StateMachine.initialSwerveRotation != null) {
            final Pose2d pose = new Pose2d(StateMachine.odometryAndVision.getEstimatedPose().getTranslation(), StateMachine.initialSwerveRotation.plus(offset));
            StateMachine.odometryAndVision.resetPose(pose);
        }
    }
    public void resetGyro() {
        resetGyro(Rotation2d.kZero);
    }

    public void resetPose(Pose2d pose) {
        System.out.println("resetPose called");
        StateMachine.odometryAndVision.resetPose(pose);
    }

    private Command repeatingSimFuelCommand() {
        return cmdName(Commands.repeatingSequence(
            Commands.either(Commands.runOnce(() -> m_commandScheduler.schedule(new SimulationCommands.SimFuelCommand(
                () -> new Pose3d(StateMachine.odometryAndVision.getEstimatedPose())
                    .plus(new Transform3d(0, 0, Units.feetToMeters(1.5d), Rotation3d.kZero)),
                () -> {
                    Pose2d robot = StateMachine.odometryAndVision.getEstimatedPose();
                    LinearVelocity vel = m_shooterSubsystem.getExitVelocity();
                    Logger.recordOutput("SimFuel/ShooterExitVelocity", vel);

                    Angle turretAngle = Constants.USE_TURRET ? m_turretSubsystem.getAngle() : Degrees.of(0d);
                    Rotation2d fieldYaw = robot.getRotation().plus(new Rotation2d(turretAngle.in(Radians)));
                    double pitchRad = m_shooterSubsystem.getHoodAngle().plus(mechanismPositionToAngle(hoodPositionHome)).in(Radians);

                    Translation3d localDirection = new Translation3d(
                        Math.cos(pitchRad),
                        0d,
                        Math.sin(pitchRad)
                    );
                    Rotation3d fieldRotation = new Rotation3d(
                        0d,
                        0d,
                        fieldYaw.getRadians()
                    );

                    Translation3d fieldDirection = localDirection.rotateBy(fieldRotation);
                    fieldDirection = fieldDirection.times(vel.in(MetersPerSecond));
                    // is this maliciously ensuring "works in sim"??? idk
                    fieldDirection = fieldDirection.plus(ConversionUtil.chassisSpeedsToTranslation3d(m_driveSubsystem.getChassisSpeeds()));

                    return fieldDirection;
                }
            ))), Commands.none(), m_indexerSubsystem::getIsFeeding),
            Commands.waitSeconds(0.002d)
        ), "RepeatingSimFuel");
    }
}
