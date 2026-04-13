package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;


import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public final class ShooterConstants {
    // public static final long spinUpDurationMilliseconds = 800L;
    public static final double shouldIndexFlywheelVelocityThresholdRPS = 1.5d; // 5.5
    public static final double shouldIndexKickerVelocityThresholdRPS = 1.5d;
    // public static final double shouldIndexKickerVelocityThresholdSeconds = 0.5d;
    // public static final double shooterReverseCountSeconds = 0.1d;
    public static final double latencyCompensationSeconds = 0.150d;
    
    public static final int flywheelMotorIdTopLeft = 17;
    public static final int flywheelMotorIdBottomLeft = 18;
    public static final int flywheelMotorIdTopRight = 19;
    public static final int flywheelMotorIdBottomRight = 20;
    public static final int flywheelCurrentLimit = 20; // 15; 18
    public static final double flywheelReduction = (24d / 18d);
    public static final NeutralModeValue flywheelNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue flywheelInvertedTopLeft = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue flywheelInvertedBottomLeft = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue flywheelInvertedTopRight = InvertedValue.Clockwise_Positive;
    public static final InvertedValue flywheelInvertedBottomRight = InvertedValue.Clockwise_Positive;
    // public static final AngularVelocity flywheelVelocityReverse = RPM.of(-2000d);

    public static final double flywheelPidP = 0.7d; // 0.2
    public static final double flywheelPidV = 12d / (5800d / 60d);
    public static final double flywheelTargetAcceleration = 266d;
    public static final Distance flywheelRadiusBig = Inches.of(3d);
    public static final Distance flywheelRadiusSmall = Inches.of(1d);

    public static final int hoodMotorId = 21;
    // public static final double hoodCurrentLimit = 40;
    public static final double hoodMotorReduction = (50d / 10d) * (54d / 22d) * (160d / 10d);
    public static final NeutralModeValue hoodNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue hoodInverted = InvertedValue.Clockwise_Positive;

    public static final double hoodPositionHome = angleToMechanismPosition(Degrees.of(0d));
    public static final double hoodPositionMax = angleToMechanismPosition(Degrees.of(50d)); // 37.55

    public static final double hoodPidP = 300d;
    public static final double hoodTargetAcceleration = 250d;
    public static final double hoodMaxVelocity = 10d;

    public static final double hoodAutoStowSeconds = 0.25d;

    public static final double hoodZeroDutyCycle = -0.07d;
    public static final double hoodZeroVelocityThresholdRPS = 0.00002d;
    
    // shoot -> spin up upper kicker, wait for speed, then spin lower and top roller same time
    public static final int upperKickerMotorId = 22;
    public static final int upperKickerCurrentLimit = 40; // 40; 35
    public static final NeutralModeValue upperKickerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue upperKickerInverted = InvertedValue.CounterClockwise_Positive;
    public static final AngularVelocity upperKickerVelocity = RPM.of(4000d);
    public static final AngularVelocity upperKickerVelocityReverse = upperKickerVelocity.unaryMinus();
    //
    public static final double upperKickerPidP = 0.6d;
    public static final double upperKickerPidV = 12d / (5800d / 60d);
    public static final double upperKickerTargetAcceleration = 266d;

    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
    public static record InterpolationShooterParams(double rpm, double degrees) { };

    public static final InterpolationShooterParams allianceFeedParamsNeutralZone =
        new InterpolationShooterParams(3000d, 48d);
    public static final InterpolationShooterParams allianceFeedParamsFar =
        new InterpolationShooterParams(4000d, 48d);

    private static final InterpolationShooterParams trenchShooterParams =
        new InterpolationShooterParams(2800d, 18.75d);
    public static final InterpolationShooterParams hubCenterParams =
        new InterpolationShooterParams(2150d, 7d);

    public static enum InterpolationParamMap {
        Normal;
        // Low;

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
            // Normal.map.put(1.78, hubCenterParams);
            // Normal.map.put(3.1, new InterpolationShooterParams(2800d, 15d)); // tower left
            // // Normal.map.put(3.63, trenchShooterParams);
            // Normal.map.put(3.38, trenchShooterParams);
            // Normal.map.put(4.14, new InterpolationShooterParams(3025d, 18.75d));
            // Normal.map.put(5.47, new InterpolationShooterParams(3200d, 27d)); // field left

            // half a meter increments (in theory)
            Normal.map.put(1.76d, hubCenterParams);
            Normal.map.put(2.25d, new InterpolationShooterParams(2300d, 10d));
            Normal.map.put(2.75d, new InterpolationShooterParams(2450d, 12d));
            Normal.map.put(3.26d, new InterpolationShooterParams(2600d, 15d));
            Normal.map.put(3.79d, new InterpolationShooterParams(2725d, 17d));
            Normal.map.put(4.25d, new InterpolationShooterParams(2900d, 19d));
            Normal.map.put(4.75d, new InterpolationShooterParams(3000d, 21d));
            Normal.map.put(5.00d, new InterpolationShooterParams(3200d, 21.5d));
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
