package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.state_machine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import java.util.Comparator;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class IntakeCommands {
    private static final double lookAheadSeconds = 0.1d;
    private static final double intakingOffset = 0d;
    private static final double fuelMaxDistance = 4d;
    private static final double fuelMaxXDistance = 1d;
    private static final double fuelMinXDistance = 0.1d;
    private static final double fuelMaxYDistance = 1d;
    private static final double fuelJAMaxDistance = 0.6d;
    private static final double fuelMaxAngleDegrees = 60d;
    private static final double angleDifferenceWeight = 0.3d;
    private static final double driveMinOutput = 0.1d;

    private static final PIDController driveController = new PIDController(1d, 0d, 0d);

    public static Command joystickAssist(
            Drive drive,
            DoubleSupplier driverX,
            DoubleSupplier driverY,
            DoubleSupplier driverOmega,
            BooleanSupplier robotRelative) {
        return drive.run(() -> {
            Translation2d linearVelocity =
                    DriveCommands.getLinearVelocityFromJoysticks(driverX.getAsDouble(), driverY.getAsDouble());
            ChassisSpeeds wantedSpeeds = DriveCommands.getChassisSpeedsFromJoysticks(
                    drive, driverX.getAsDouble(), driverY.getAsDouble(), driverOmega.getAsDouble(), Optional.empty());
            wantedSpeeds = robotRelative.getAsBoolean()
                    ? wantedSpeeds
                    : ChassisSpeeds.fromFieldRelativeSpeeds(
                            wantedSpeeds,
                            StateMachine.ALLIANCE == Alliance.Red
                                    ? StateMachine.odometryAndVision
                                            .getRotation()
                                            .plus(Rotation2d.kPi)
                                    : StateMachine.odometryAndVision.getRotation());
            // Find nearest fuel
            var fuelTranslation = getNearestFuel();
            // Log targeted fuel
            Logger.recordOutput(
                    "IntakeCommands/TargetedFuel",
                    fuelTranslation
                            .map(fuel -> new Translation3d[] {
                                new Translation3d(fuel.getX(), fuel.getY(), FieldConstants.FUEL_DIAMETER.in(Meters))
                            })
                            .orElseGet(() -> new Translation3d[] {}));
            if (fuelTranslation.isEmpty()) {
                Logger.recordOutput("IntakeCommands/JA/WantedSpeeds", wantedSpeeds);
                driveController.reset();
                drive.runVelocity(wantedSpeeds);
                return;
            }

            // Nudge wanted speeds in direction of fuel
            Pose2d robot = StateMachine.odometryAndVision.getEstimatedPose();
            Transform2d robotToIntake =
                    new Transform2d(-(DriveConstants.BUMPER_WIDTH.in(Meters) + intakingOffset), 0d, Rotation2d.kPi);
            Pose2d intakePose = robot.plus(robotToIntake);
            Pose2d predictedIntakePose = intakePose.exp(wantedSpeeds.toTwist2d(lookAheadSeconds));
            Logger.recordOutput("IntakeCommands/PredictedPose", predictedIntakePose);
            Translation2d intakeToFuelError = new Pose2d(fuelTranslation.get(), intakePose.getRotation())
                    .relativeTo(intakePose)
                    .getTranslation();
            final double driveError = intakeToFuelError.getNorm();
            boolean shouldDrive = true;
            if (Math.abs(intakeToFuelError.getY()) >= fuelMaxYDistance
                    || Math.abs(intakeToFuelError.getX()) >= fuelMaxXDistance
                    || predictedIntakePose.getTranslation().getDistance(fuelTranslation.get()) >= fuelJAMaxDistance) {
                driveController.reset();
                shouldDrive = false;
            }

            Translation2d driveVelocity = new Pose2d(
                            new Translation2d(
                                    shouldDrive ? driveController.calculate(driveError, 0d) : 0d,
                                    intakeToFuelError.getAngle()),
                            intakeToFuelError.getAngle())
                    .transformBy(robotToIntake.inverse())
                    .getTranslation();
            wantedSpeeds = new ChassisSpeeds(
                    wantedSpeeds.vxMetersPerSecond
                            + (intakeToFuelError.getX() <= 0.1
                                            ? 0d
                                            : driveVelocity.getX()
                                                    * MathUtil.clamp(
                                                            (driveError - fuelMinXDistance) / (fuelMinXDistance + 1d),
                                                            0d,
                                                            1d))
                                    * linearVelocity.getNorm(),
                    wantedSpeeds.vyMetersPerSecond
                            + driveVelocity.getY() * MathUtil.clamp(linearVelocity.getNorm() + driveMinOutput, 0d, 1d),
                    wantedSpeeds.omegaRadiansPerSecond);
            Logger.recordOutput("IntakeCommands/JA/WantedSpeeds", wantedSpeeds);
            drive.runVelocity(wantedSpeeds);
        });
    }

    private static Optional<Translation2d> getNearestFuel() {
        Pose2d robot = StateMachine.odometryAndVision.getEstimatedPose();
        Pose2d robotFlipped = robot.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.kPi));
        Pose2d intakePose = robot.transformBy(
                new Transform2d(-(DriveConstants.BUMPER_WIDTH.in(Meters) + intakingOffset), 0d, Rotation2d.kPi));
        ChassisSpeeds robotVelocity = Drive.instance.getChassisSpeeds();
        Pose2d predictedRobot = robot.exp(robotVelocity.toTwist2d(lookAheadSeconds));
        Pose2d predictedIntakePose = predictedRobot.transformBy(
                new Transform2d(-(DriveConstants.BUMPER_WIDTH.in(Meters) + intakingOffset), 0d, Rotation2d.kPi));
        Logger.recordOutput("IntakeCommands/PredictedRobot", predictedRobot);

        return StateMachine.odometryAndVision.getFuelTranslations().stream()
                .filter(fuel -> fuel.getDistance(intakePose.getTranslation()) <= fuelMaxDistance
                        && Math.abs(robotFlipped
                                        .getRotation()
                                        .minus(fuel.minus(robotFlipped.getTranslation())
                                                .getAngle())
                                        .getDegrees())
                                <= fuelMaxAngleDegrees)
                .min(Comparator.comparingDouble(fuel -> fuel.getDistance(predictedIntakePose.getTranslation())
                        + Math.abs(fuel.minus(robotFlipped.getTranslation())
                                        .getAngle()
                                        .minus(robotFlipped.getRotation())
                                        .getRadians()
                                * angleDifferenceWeight)));
    }
}
