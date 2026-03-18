package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import java.util.HashMap;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.state_machine.StateMachine;

public final class ShooterConstants {
    // public static final long spinUpDurationMilliseconds = 800L;
    public static final double shouldIndexFlywheelVelocityThresholdRPS = 5.5d;
    // public static final double shouldIndexKickerVelocityThresholdRPS = 3d;
    public static final double shouldIndexKickerVelocityThresholdSeconds = 0.5d;
    public static final double latencyCompensationSeconds = 0.150d;
    
    public static final int flywheelMotorIdLeft = 1;
    public static final int flywheelMotorIdMiddleUpper = 12;
    public static final int flywheelMotorIdMiddleLower = 17;
    public static final int flywheelMotorIdRight = 16;
    public static final int flywheelCurrentLimit = 15;
    public static final NeutralModeValue flywheelNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue flywheelInvertedLeft = InvertedValue.Clockwise_Positive;
    public static final InvertedValue flywheelInvertedMiddleUpper = InvertedValue.Clockwise_Positive;
    public static final InvertedValue flywheelInvertedMiddleLower = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue flywheelInvertedRight = InvertedValue.CounterClockwise_Positive;
    public static final boolean flywheelFOC = true;
    // public static final AngularVelocity flywheelVelocityReverse = RPM.of(-2000d);

    public static final double flywheelPidP = 0.5d;
    public static final double flywheelFreeSpeed = flywheelFOC ? 5800d : 6000d;
    public static final double flywheelPidV = 12d / (flywheelFreeSpeed / 60d);
    public static final double flywheelTargetAcceleration = 266d;
    public static final Distance flywheelRadiusBig = Inches.of(2d);
    public static final Distance flywheelRadiusSmall = Inches.of(1d);

    public static final int hoodMotorId = 58;
    // public static final double hoodCurrentLimit = 40;
    public static final double hoodMotorReduction = (50d / 12d) * (30d / 16d) * (155d / 10d);
    public static final NeutralModeValue hoodNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue hoodInverted = InvertedValue.CounterClockwise_Positive;

    public static final double hoodPositionHome = angleToMechanismPosition(Degrees.of(20d));
    public static final double hoodPositionMax = hoodPositionHome + angleToMechanismPosition(Degrees.of(23.4d));

    public static final double hoodPidP = 300d;
    public static final double hoodTargetAcceleration = 250d;
    public static final double hoodMaxVelocity = 10d;
    
    public static final int kickerMotorId = 9;
    public static final int kickerCurrentLimit = 40;
    public static final NeutralModeValue kickerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue kickerInverted = InvertedValue.CounterClockwise_Positive;
    public static final AngularVelocity kickerVelocity = RPM.of(4500d);
    public static final AngularVelocity kickerVelocityReverse = kickerVelocity.unaryMinus();

    public static final double kickerPidP = 0.62d;
    public static final double kickerPidV = 12d / (5800d / 60d);
    public static final double kickerTargetAcceleration = 266d;

    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
    public static record InterpolationShooterParams(double rpm, double degrees) { };

    public static final InterpolationShooterParams allianceFeedParams =
        new InterpolationShooterParams(1700d, 40d);
    private static final InterpolationShooterParams trenchShooterParams =
        new InterpolationShooterParams(1750d, 30d);
    public static final InterpolationShooterParams hubCenterParams =
        new InterpolationShooterParams(1435d, 20d);
    // public static enum LauncherLocationParam {
    //     TrenchLeft(
    //         new Translation2d(Meters.of(4.138d), Meters.of(7.632d)),
    //         trenchShooterParams),
    //     TrenchRight(
    //         new Translation2d(Meters.of(4.138d), Meters.of(0.444d)),
    //         trenchShooterParams),
    //     HubCenter(
    //         new Translation2d(Meters.of(2.985594d), StateMachine.getHubPoseBlue().getMeasureY()),
    //         hubCenterParams,
    //     TowerLeft(
    //         new Translation2d(Meters.of(1.52d), Meters.of(4.177d)),
    //         new InterpolationShooterParams(1600d, 30d)),
    //     FieldLeft(
    //         new Translation2d(Meters.of(0.456d), Meters.of(7.633d)),
    //         new InterpolationShooterParams(2200d, 30d));

    //     private Translation2d m_location;
    //     public final InterpolationShooterParams m_params;

    //     LauncherLocationParam(Translation2d location, InterpolationShooterParams params) {
    //         m_location = location;
    //         m_params = params;
    //     }

    //     static Optional<LauncherLocationParam> getFromRobot(Translation2d robot) {
    //         double furthestDistance = 9999d;
    //         LauncherLocationParam result = null;
    //         final var values = LauncherLocationParam.values();
    //         for (int i = 0; i < values.length; i++) {
    //             final var value = values[i];
    //             final double distance = robot.getDistance(StateMachine.allianceFlip(value.m_location));
    //             // if we're closer than the previous point and we're not within a few inches to the last distance (avoid being between and constantly flipping between two)
    //             if (distance < furthestDistance && (furthestDistance == 9999d ? true : Math.abs(distance - furthestDistance) > Units.inchesToMeters(4d))) {
    //                 furthestDistance = distance;
    //                 result = value;
    //             }
    //         }

    //         if (result == null) {
    //             Logger.recordOutput("Shooter/ClosestParamLocation", "[NONE]");
    //             return Optional.empty();
    //         }
    //         Logger.recordOutput("Shooter/ClosestParamLocation", result.name());
    //         return Optional.of(result);
    //     }
    // }

    public static enum InterpolationParamMap {
        Normal,
        Low;

        public final InterpolatingTreeMap<Double, InterpolationShooterParams> map =
            new InterpolatingTreeMap<Double, InterpolationShooterParams>(
                InverseInterpolator.forDouble(),
                (InterpolationShooterParams start, InterpolationShooterParams end, double q) ->
                    new InterpolationShooterParams(
                        MathUtil.interpolate(start.rpm, end.rpm, q),
                        MathUtil.interpolate(end.degrees, end.degrees, q)
                    )
            );

        static {
            // Normal.map.put(1d, new InterpolationShooterParams(3000d, 60d));
            // Normal.map.put(2d, new InterpolationShooterParams(3000d, 60d));

            // for (final var value : LauncherLocationParam.values()) {
            //     final double distance = value.m_location.getDistance(StateMachine.getHubPoseBlue().toTranslation2d());
            //     System.out.println(value.name() + "; " + distance);
            //     Normal.map.put(distance, value.m_params);
            // }

            Normal.map.put(1.65, hubCenterParams);
            Normal.map.put(3.1, new InterpolationShooterParams(1600d, 30d)); // tower left
            Normal.map.put(3.63, trenchShooterParams);
            Normal.map.put(5.5, new InterpolationShooterParams(2200d, 30d)); // field left
            // cross map (only x direction so not horizontal) alliance feed
            Normal.map.put(11d, new InterpolationShooterParams(3000d, mechanismPositionToAngle(hoodPositionHome).in(Degrees)));

            // TODO: low map, if we want to use it
        }
    }
    public static final InterpolationParamMap paramMap = InterpolationParamMap.Normal;
    // distance to seconds
    public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    static {
        timeOfFlightMap.put(1.65d, 0.9291666666666675d); // hub center
        timeOfFlightMap.put(3.63d, 1.12777777777d); // trench (MY BEST GUESS; PLEASE CHANGE)
    }
}
