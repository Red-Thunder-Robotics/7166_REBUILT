package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.ConversionUtil.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public final class GroundIntakeConstants {
    public static final int rollerMotorId = 12;
    public static final int rollerCurrentLimit = 45;
    public static final int rollerCurrentLimitAuto = 45;
    public static final NeutralModeValue rollerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue rollerInverted = InvertedValue.CounterClockwise_Positive;
    public static final Distance rollerCircumference = Meters.of(0.11172d);
    public static final AngularVelocity rollerOutputVelocityMinimum = RPM.of(2000d);
    public static final AngularVelocity rollerOutputVelocity = RPM.of(3500d);
    public static final AngularVelocity rollerOutputVelocityReverse = rollerOutputVelocity.unaryMinus();

    public static final double rollerPidP = 0.3d;
    public static final double rollerPidV = 12d / (5800d / 60d);
    public static final double rollerTargetAcceleration = 266d;

    public static final int actuatorMotorId = 13;
    public static final int actuatorCurrentLimit = 30;
    public static final double actuatorMotorReduction = (50d / 10d) * (60d / 14d) * (54d / 22d);
    public static final NeutralModeValue actuatorNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue actuatorInverted = InvertedValue.CounterClockwise_Positive;
    public static final double actuatorPositionHome = angleToMechanismPosition(Degrees.of(-102d));
    public static final double actuatorPositionDeployed = angleToMechanismPosition(Degrees.of(0d));
    public static final double actuatorPositionOscillate = actuatorPositionHome / 2d;

    public static final double actuatorPidP = 40d;
    public static final double actuatorTargetAcceleration = 50d;
    public static final double actuatorMaxVelocity = 5d;
}
