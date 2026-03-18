package frc.robot.subsystems.climbermark1;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.util.ConversionUtil.distanceToMechanismPosition;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;

public final class ClimberConstants {
    // public static final int leftMotorId = -1;
    // public static final double leftCurrentLimit = 40;
    // public static final InvertedValue leftInverted = InvertedValue.Clockwise_Positive;
    
    public static final int rightMotorId = 31;
    public static final double rightCurrentLimit = 40;
    public static final InvertedValue rightInverted = InvertedValue.CounterClockwise_Positive;
    
    public static final double motorReduction = 30d + (2/3d);
    public static final Distance pitchCircumference = Millimeters.of(120d);
    public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;

    public static final double positionHome = distanceToMechanismPosition(Inches.of(0d), pitchCircumference);
    public static final double positionDeployed = distanceToMechanismPosition(Inches.of(8.25d), pitchCircumference);

    public static final double pidP = 20d;
    public static final double targetAcceleration = 70d;
    public static final double maxVelocity = 5d;
}
