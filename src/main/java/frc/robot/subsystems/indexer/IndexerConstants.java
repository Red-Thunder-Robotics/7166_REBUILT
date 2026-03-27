package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public final class IndexerConstants {
    public static final int indexerMotorId = 14;
    public static final int indexerCurrentLimit = 25; // 30
    public static final double indexerMotorReduction = (18d / 12d);
    public static final NeutralModeValue indexerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue indexerInverted = InvertedValue.CounterClockwise_Positive;
    public static final AngularVelocity indexerOutputVelocity = RPM.of(3500d);
    public static final AngularVelocity indexerOutputVelocityReverse = RPM.of(-1000d);

    public static final double indexerPidP = 0.9d;
    public static final double indexerPidV = 12d / (5800d / 60d);
    public static final double indexerTargetAcceleration = 266d;

    public static final int topRollerMotorId = 15;
    public static final int topRollerCurrentLimit = 40; // 40; 35
    public static final NeutralModeValue topRollerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue topRollerInverted = InvertedValue.CounterClockwise_Positive;
    public static final double topRollerReduction = (30d / 24d);
    public static final AngularVelocity topRollerVelocity = RPM.of(2000d);
    public static final AngularVelocity topRollerVelocityReverse = RPM.of(-1000d);
    //
    public static final double topRollerPidP = 0.8d;
    public static final double topRollerPidV = 12d / (5800d / 60d);
    public static final double topRollerTargetAcceleration = 266d;

    public static final int lowerKickerMotorId = 16;
    public static final int lowerKickerCurrentLimit = 25; // 40; 20
    public static final NeutralModeValue lowerKickerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue lowerKickerInverted = InvertedValue.Clockwise_Positive;
    public static final AngularVelocity lowerKickerVelocity = RPM.of(3500d);
    public static final AngularVelocity lowerKickerVelocityReverse = lowerKickerVelocity.unaryMinus();

    public static final double lowerKickerVelocityThresholdRPS = 1.5d;

    //
    public static final double lowerKickerPidP = 0.1d;
    public static final double lowerKickerPidV = 12d / (5800d / 60d);
    public static final double lowerKickerTargetAcceleration = 5d;
}
