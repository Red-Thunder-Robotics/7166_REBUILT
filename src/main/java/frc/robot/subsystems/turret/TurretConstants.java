package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.util.ConversionUtil.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public final class TurretConstants {
    public static final int motorId = -1;
    public static final double currentLimit = 40;
    public static final double motorReduction = 1d;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;

    public static final double positionMin = angleToMechanismPosition(Degrees.of(-200d));
    public static final double positionMax = angleToMechanismPosition(Degrees.of(200d));
    public static final double positionHome = angleToMechanismPosition(Degrees.of(0d));

    public static final double pidP = 0d;
    public static final double targetAcceleration = 0d;
    public static final double maxVelocity = 0d;

    public static final double shouldIndexThresholdDegrees = 5d;

    public static final Transform3d robotToTurretTransform =
            new Transform3d(new Translation3d(Inches.zero(), Inches.zero(), Inches.of(4d)), Rotation3d.kZero);
    public static final Distance distanceAboveFunnel = Inches.of(20); // how high to clear the funnel
}
