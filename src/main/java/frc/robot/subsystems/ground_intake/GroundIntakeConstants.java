package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.ConversionUtil.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public final class GroundIntakeConstants {
    public static final int rightRollerMotorId = 23;
    public static final int rollerCurrentLimit = 25; // 45
    public static final int rollerCurrentLimitAuto = 40; // 30
    public static final double rollerMotorReduction = (18d / 12d);
    public static final NeutralModeValue rollerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue rightRollerInverted = InvertedValue.CounterClockwise_Positive;

    public static final int leftRollerMotorId = 12;
    public static final MotorAlignmentValue leftRollerMotorAlignment = MotorAlignmentValue.Opposed;

    public static final Distance rollerCircumference = Meters.of(0.11172d);
    public static final boolean rollerOutputVelocityUsesChassisSpeeds = false;
    public static final boolean rollerOutputUsesVelocityControl = true;

    public static final AngularVelocity rollerOutputVelocityMinimum = RPM.of(2000d);
    public static final AngularVelocity rollerOutputVelocity = RPM.of(3850d); // 3500; 5800
    public static final AngularVelocity rollerOutputVelocityAuto = RPM.of(3850d); // 4500; 5000; 5500; 5000; 5800
    public static final AngularVelocity rollerOutputVelocityReverse = rollerOutputVelocity.unaryMinus();
    public static final AngularVelocity rollerOutputVelocityHalfway = RPM.of(500d);
    public static final Current rollerOutputCurrent = Amps.of(25d);
    public static final Current rollerOutputCurrentReverse = rollerOutputCurrent.unaryMinus();

    public static final double rollerPidP = 0.75d;
    public static final double rollerPidV = 12d / (5800d / 60d);
    public static final double rollerTargetAcceleration = 266d;

    public static final int actuatorMotorId = 13;
    public static final int actuatorCurrentLimit = 30;
    public static final double actuatorMotorReduction = (50d / 10d) * (60d / 14d) * (54d / 22d);
    public static final NeutralModeValue actuatorNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue actuatorInverted = InvertedValue.CounterClockwise_Positive;
    public static final double actuatorPositionHome = angleToMechanismPosition(Degrees.of(-92d)); // -101
    public static final double actuatorPositionDeployed = angleToMechanismPosition(Degrees.of(0d));
    public static final double actuatorPositionOscillate = actuatorPositionHome / 2d;

    public static final double actuatorPidP = 200d; // 40
    // public static final double actuatorTargetAcceleration = 50d;
    public static final double actuatorTargetAcceleration = 20d;
    public static final double actuatorMaxVelocity = 0.5d; // 5

    public static final double actuatorZeroDutyCycle = -0.065d;
    public static final double actuatorZeroVelocityThresholdRPS = 0.00002d;
}
