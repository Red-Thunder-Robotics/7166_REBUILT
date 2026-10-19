// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found at
// https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/LICENSE
//
// Ported from 6328's RobotCode2026Public, src/main/java/org/littletonrobotics/frc2026/util/BeachedUtil.java
// (commit b2923d6). Changes for 7166: the robot rotation comes from Drive's Pigeon roll and pitch
// instead of 6328's RobotState, the speed limits use TunerConstants, and lombok's @Setter is
// written out.

package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class BeachedUtil {
    private static BeachedUtil instance;

    private final double rotationThreshold = Units.degreesToRadians(3);
    private final double minTilt = Units.degreesToRadians(3);
    private final double maxTilt = Units.degreesToRadians(15);
    private final double minSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.4;
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private BooleanSupplier ignoreBeached = () -> false;

    public static BeachedUtil getInstance() {
        if (instance == null) instance = new BeachedUtil();
        return instance;
    }

    public void setIgnoreBeached(BooleanSupplier ignoreBeached) {
        this.ignoreBeached = ignoreBeached;
    }

    /**
     * Gets the speeds to unbeach the robot
     *
     * @return Optional ChassisSpeeds in robot relative coordinates
     */
    public Optional<ChassisSpeeds> getBeachedSpeeds() {
        if (ignoreBeached.getAsBoolean()) return Optional.empty();

        Optional<Rotation3d> robotRotation = Drive.instance == null
            ? Optional.empty()
            : Optional.of(new Rotation3d(Drive.instance.getRollRad(), Drive.instance.getPitchRad(), 0.0));

        if (robotRotation.isPresent()) {
            // Define gravity in the field frame (straight down)
            Translation3d fieldGravity = new Translation3d(0.0, 0.0, -1.0);

            // Rotate gravity into the robot's coordinate frame using the inverse rotation
            Translation3d robotGravity = fieldGravity.rotateBy(robotRotation.get().unaryMinus());

            // Calculate exact tilt angle from horizontal using the Z component
            double tilt = Math.acos(Math.abs(robotGravity.getZ()));

            // Check threshold
            if (tilt < rotationThreshold) {
                return Optional.empty();
            }

            // Extract 2D downhill direction directly from X and Y
            Rotation2d beachedDirection = new Rotation2d(-robotGravity.getX(), -robotGravity.getY());

            // Calculate the velocity
            double clampedTilt = MathUtil.clamp(tilt, minTilt, maxTilt);
            double t = MathUtil.inverseInterpolate(minTilt, maxTilt, clampedTilt);
            double velocity = MathUtil.interpolate(minSpeed, maxSpeed, t);

            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    beachedDirection.getCos() * velocity, beachedDirection.getSin() * velocity, 0);

            return Optional.of(speeds);
        } else {
            return Optional.empty();
        }
    }
}
