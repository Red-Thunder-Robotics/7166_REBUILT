package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public final class IndexerConstants {
    public static final int indexerMotorId = 56;
    public static final int indexerCurrentLimit = 30;
    public static final double indexerMotorReduction = (18d / 12d);
    public static final NeutralModeValue indexerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue indexerInverted = InvertedValue.CounterClockwise_Positive;
    public static final AngularVelocity indexerOutputVelocity = RPM.of(3500d);
    public static final AngularVelocity indexerOutputVelocityReverse = indexerOutputVelocity.unaryMinus();

    public static final double indexerPidP = 1.38d;
    public static final double indexerPidV = 12d / (5800d / 60d);
    public static final double indexerTargetAcceleration = 266d;

    public static final int topRollerMotorId = 9;
    public static final int topRollerCurrentLimit = 40;
    public static final NeutralModeValue topRollerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue topRollerInverted = InvertedValue.CounterClockwise_Positive;
    public static final double topRollerReduction = (36d / 18d);
    public static final AngularVelocity topRollerVelocity = RPM.of(3500d);
    public static final AngularVelocity topRollerVelocityReverse = topRollerVelocity.unaryMinus();
    //
    public static final double topRollerPidP = 0d;
    public static final double topRollerPidV = 12d / (5800d / 60d);
    public static final double topRollerTargetAcceleration = 5d;

    public static final int lowerKickerMotorId = 9;
    public static final int lowerKickerCurrentLimit = 40;
    public static final NeutralModeValue lowerKickerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue lowerKickerInverted = InvertedValue.Clockwise_Positive;
    public static final AngularVelocity lowerKickerVelocity = RPM.of(3500d);
    public static final AngularVelocity lowerKickerVelocityReverse = RPM.of(3500d);
    //
    public static final double lowerKickerPidP = 0d;
    public static final double lowerKickerPidV = 12d / (5800d / 60d);
    public static final double lowerKickerTargetAcceleration = 5d;
}
