package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.state_machine.StateMachine;
import frc.robot.subsystems.drive.Drive;

public final class AutoDriveCommands {
    public static Command faceRotation2d(Rotation2d target) {
        var controller = new PIDController(DriveConstants.ANGLE_KP, 0d, DriveConstants.ANGLE_KD);

        return Drive.instance
                .runOnce(() -> {
                    controller.reset();
                    controller.setSetpoint(target.getRadians());
                })
                .andThen(Commands.run(() -> {
                            Drive.instance.runVelocity(new ChassisSpeeds(
                                    0d,
                                    0d,
                                    controller.calculate(StateMachine.odometryAndVision
                                            .getRotation()
                                            .getRadians())));
                        })
                        .until(() -> {
                            return controller.getError() < Units.degreesToRadians(1d)
                                    && controller.getErrorDerivative() < Units.degreesToRadians(4d);
                        }))
                .finallyDo(controller::close);
    }

    public static Command driveToPose(Pose2d target) {
        var controllerX = new PIDController(DriveConstants.TRANSLATION_KP, 0d, 0d);
        var controllerY = new PIDController(DriveConstants.TRANSLATION_KP, 0d, 0d);

        return Drive.instance
                .runOnce(() -> {
                    controllerX.reset();
                    controllerY.reset();
                    controllerX.setSetpoint(target.getX());
                    controllerY.setSetpoint(target.getY());
                })
                .andThen(Commands.run(() -> {
                            Drive.instance.runVelocity(new ChassisSpeeds(
                                    controllerX.calculate(StateMachine.odometryAndVision
                                            .getEstimatedPose()
                                            .getX()),
                                    controllerY.calculate(StateMachine.odometryAndVision
                                            .getEstimatedPose()
                                            .getY()),
                                    0d));
                        })
                        .until(() -> {
                            final boolean x = controllerX.getError() < Units.inchesToMeters(1d)
                                    && controllerX.getErrorDerivative() < Units.inchesToMeters(0.5d);
                            final boolean y = controllerY.getError() < Units.inchesToMeters(1d)
                                    && controllerY.getErrorDerivative() < Units.inchesToMeters(0.5d);
                            return x && y;
                        }))
                .finallyDo(() -> {
                    controllerX.close();
                    controllerY.close();
                });
    }
}
