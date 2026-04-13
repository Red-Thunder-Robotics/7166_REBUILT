package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.ConversionUtil.angleToMechanismPosition;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.PhoenixUtil.MotorAction;

public final class ShooterIOReal implements ShooterIO {
    private final TalonFX m_flywheelMotorTopLeft = new TalonFX(flywheelMotorIdTopLeft, Constants.CANBUS);
    private final TalonFX m_flywheelMotorBottomLeft = new TalonFX(flywheelMotorIdBottomLeft, Constants.CANBUS);
    private final TalonFX m_flywheelMotorTopRight = new TalonFX(flywheelMotorIdTopRight, Constants.CANBUS);
    private final TalonFX m_flywheelMotorBottomRight = new TalonFX(flywheelMotorIdBottomRight, Constants.CANBUS);
    private final TalonFX m_hoodMotor = new TalonFX(hoodMotorId, Constants.CANBUS);
    private final TalonFX m_upperKickerMotor = new TalonFX(upperKickerMotorId, Constants.CANBUS);
    
    private double m_hoodTargetPosition = hoodPositionHome;

    private final StatusSignal<AngularVelocity> m_flywheelTopLeftVelocitySignal = m_flywheelMotorTopLeft.getVelocity();
    private final StatusSignal<Current> m_flywheelTopLeftCurrentSignal = m_flywheelMotorTopLeft.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_flywheelBottomLeftVelocitySignal = m_flywheelMotorBottomLeft.getVelocity();
    private final StatusSignal<Current> m_flywheelBottomLeftCurrentSignal = m_flywheelMotorBottomLeft.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_flywheelTopRightVelocitySignal = m_flywheelMotorTopRight.getVelocity();
    private final StatusSignal<Current> m_flywheelTopRightCurrentSignal = m_flywheelMotorTopRight.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_flywheelBottomRightVelocitySignal = m_flywheelMotorBottomRight.getVelocity();
    private final StatusSignal<Current> m_flywheelBottomRightCurrentSignal = m_flywheelMotorBottomRight.getSupplyCurrent();

    private final StatusSignal<Angle> m_hoodPositionSignal = m_hoodMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_hoodVelocitySignal = m_hoodMotor.getVelocity();
    private final StatusSignal<Current> m_hoodCurrentSignal = m_hoodMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_upperKickerVelocitySignal = m_upperKickerMotor.getVelocity();
    private final StatusSignal<Current> m_upperKickerCurrentSignal = m_upperKickerMotor.getSupplyCurrent();

    private final MotionMagicVelocityVoltage m_flywheelVelocityRequest = new MotionMagicVelocityVoltage(0d)
        .withEnableFOC(true);
    private final MotionMagicVoltage m_hoodPositionRequest = new MotionMagicVoltage(m_hoodTargetPosition);
        private final MotionMagicVelocityVoltage m_kickerVelocityRequest = new MotionMagicVelocityVoltage(0d)
        .withEnableFOC(true);
    private final DutyCycleOut m_hoodDutyCycle = new DutyCycleOut(hoodZeroDutyCycle);

    public ShooterIOReal() {
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = flywheelNeutralMode;

        flywheelConfig.Feedback.SensorToMechanismRatio = flywheelReduction;

        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = flywheelCurrentLimit;

        flywheelConfig.Slot0.kP = flywheelPidP;
        flywheelConfig.Slot0.kV = flywheelPidV;
        flywheelConfig.MotionMagic.MotionMagicAcceleration = flywheelTargetAcceleration;

        var hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = hoodNeutralMode;
        hoodConfig.MotorOutput.Inverted = hoodInverted;

        hoodConfig.Feedback.SensorToMechanismRatio = hoodMotorReduction;

        hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        hoodConfig.Slot0.kP = hoodPidP;

        hoodConfig.MotionMagic.MotionMagicAcceleration = hoodTargetAcceleration;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = hoodMaxVelocity;

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodPositionMax;
        // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = hoodPositionHome;

        var upperKickerConfig = new TalonFXConfiguration();
        upperKickerConfig.MotorOutput.NeutralMode = upperKickerNeutralMode;
        upperKickerConfig.MotorOutput.Inverted = upperKickerInverted;

        upperKickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        upperKickerConfig.CurrentLimits.SupplyCurrentLimit = upperKickerCurrentLimit;

        upperKickerConfig.Slot0.kP = upperKickerPidP;
        upperKickerConfig.Slot0.kV = upperKickerPidV;
        upperKickerConfig.MotionMagic.MotionMagicAcceleration = upperKickerTargetAcceleration;
        
        // PhoenixUtil.tryUntilOk(5, () -> m_upperKickerMotor.getConfigurator().apply(upperKickerConfig));
        // PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotorTopLeft.getConfigurator().apply(flywheelConfig.clone()
        //     .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedTopLeft))));
        // // FIXME: I think withInverted on all these is redundant because they're followers but I don't want to break anything and we're short on time
        // PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotorBottomLeft.getConfigurator().apply(flywheelConfig.clone()
        //     .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedBottomLeft))));
        // PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotorTopRight.getConfigurator().apply(flywheelConfig.clone()
        //     .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedTopRight))));
        // PhoenixUtil.tryUntilOk(5, () -> m_flywheelMotorBottomRight.getConfigurator().apply(flywheelConfig.clone()
        //     .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedBottomRight))));
        // PhoenixUtil.tryUntilOk(5, () -> m_hoodMotor.getConfigurator().apply(hoodConfig));
        MotorAction.configureMotor("Shooter Upper Kicker", m_upperKickerMotor, upperKickerConfig).run();
        MotorAction.configureMotor("Shooter Top Left", m_flywheelMotorTopLeft, flywheelConfig.clone()
            .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedTopLeft))).run();
        // FIXME: I think withInverted on all these is redundant because they're followers but I don't want to break anything and we're short on time
        MotorAction.configureMotor("Shooter Bottom Left", m_flywheelMotorBottomLeft, flywheelConfig.clone()
            .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedBottomLeft))).run();
        MotorAction.configureMotor("Shooter Top Right", m_flywheelMotorTopRight, flywheelConfig.clone()
            .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedTopRight))).run();
        MotorAction.configureMotor("Shooter Bottom Right", m_flywheelMotorBottomRight, flywheelConfig.clone()
            .withMotorOutput(new MotorOutputConfigs().withInverted(flywheelInvertedBottomRight))).run();
        MotorAction.configureMotor("Shooter Hood", m_hoodMotor, hoodConfig).run();

        BaseStatusSignal.setUpdateFrequencyForAll(50d,
            m_flywheelTopLeftVelocitySignal,
            m_flywheelTopLeftCurrentSignal,
            m_flywheelBottomLeftVelocitySignal,
            m_flywheelBottomLeftCurrentSignal,
            m_flywheelTopRightVelocitySignal,
            m_flywheelTopRightCurrentSignal,
            m_flywheelBottomRightVelocitySignal,
            m_flywheelBottomRightCurrentSignal,
            m_hoodPositionSignal,
            m_hoodVelocitySignal,
            m_hoodCurrentSignal,
            m_upperKickerVelocitySignal,
            m_upperKickerCurrentSignal);

        hoodZero();

        m_flywheelMotorBottomLeft.setControl(new Follower(flywheelMotorIdTopLeft, flywheelInvertedBottomLeft == flywheelInvertedTopLeft ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
        m_flywheelMotorTopRight.setControl(new Follower(flywheelMotorIdTopLeft, flywheelInvertedTopRight == flywheelInvertedTopLeft ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
        m_flywheelMotorBottomRight.setControl(new Follower(flywheelMotorIdTopLeft, flywheelInvertedBottomRight == flywheelInvertedTopLeft ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            m_flywheelTopLeftVelocitySignal,
            m_flywheelTopLeftCurrentSignal,
            m_flywheelBottomLeftVelocitySignal,
            m_flywheelBottomLeftCurrentSignal,
            m_flywheelTopRightVelocitySignal,
            m_flywheelTopRightCurrentSignal,
            m_flywheelBottomRightVelocitySignal,
            m_flywheelBottomRightCurrentSignal,
            m_hoodPositionSignal,
            m_hoodVelocitySignal,
            m_hoodCurrentSignal,
            m_upperKickerVelocitySignal,
            m_upperKickerCurrentSignal);
        
        inputs.flywheelTargetVelocityRPS = PhoenixUtil.getRequestVelocity(m_flywheelMotorTopLeft.getAppliedControl());

        final double topLeftCurrent = m_flywheelTopLeftCurrentSignal.getValueAsDouble();
        inputs.flywheelMotorTopLeftVelocityRPS = m_flywheelTopLeftVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorTopLeftCurrentAmps = topLeftCurrent;
        //
        final double bottomLeftCurrent = m_flywheelBottomLeftCurrentSignal.getValueAsDouble();
        inputs.flywheelMotorBottomLeftVelocityRPS = m_flywheelBottomLeftVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorBottomLeftCurrentAmps = bottomLeftCurrent;
        //
        final double topRightCurrent = m_flywheelTopRightCurrentSignal.getValueAsDouble();
        inputs.flywheelMotorTopRightVelocityRPS = m_flywheelTopRightVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorTopRightCurrentAmps = topRightCurrent;
        //
        final double bottomRightCurrent = m_flywheelBottomRightCurrentSignal.getValueAsDouble();
        inputs.flywheelMotorBottomRightVelocityRPS = m_flywheelBottomRightVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorBottomRightCurrentAmps = bottomRightCurrent;
        //
        inputs.flywheelTotalCurrentAmps = topLeftCurrent + bottomLeftCurrent + topRightCurrent + bottomRightCurrent;

        final double hoodTargetPosition = m_hoodTargetPosition;
        inputs.hoodTargetPositionRotations = hoodTargetPosition;
        inputs.hoodTargetPositionDegrees = mechanismPositionToAngle(hoodTargetPosition).in(Degrees);
        //
        final double hoodPositionRotations = m_hoodPositionSignal.getValueAsDouble();
        inputs.hoodPositionRotations = hoodPositionRotations;
        inputs.hoodPositionDegrees = mechanismPositionToAngle(hoodPositionRotations).in(Degrees);
        //
        inputs.hoodMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_hoodMotor.getAppliedControl());
        inputs.hoodMotorVelocityRPS = m_hoodVelocitySignal.getValueAsDouble();
        inputs.hoodMotorCurrentAmps = m_hoodCurrentSignal.getValueAsDouble();

        inputs.upperKickerTargetVelocityRPS = PhoenixUtil.getRequestVelocity(m_upperKickerMotor.getAppliedControl());
        //
        inputs.upperKickerMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_upperKickerMotor.getAppliedControl());
        inputs.upperKickerMotorVelocityRPS = m_upperKickerVelocitySignal.getValueAsDouble();
        inputs.upperKickerMotorCurrentAmps = m_upperKickerCurrentSignal.getValueAsDouble();
    }

    @Override
    public void idle() {
        m_flywheelMotorTopLeft.disable();
        m_upperKickerMotor.disable();
    }

    @Override
    public void flywheelVelocity(AngularVelocity velocity) {
        m_flywheelMotorTopLeft.setControl(m_flywheelVelocityRequest.withVelocity(velocity));
    }
    @Override
    public void flywheelStop() {
        m_flywheelMotorTopLeft.disable();
    }

    @Override
    public void hoodAngle(double position) {
        m_hoodTargetPosition = position;
        m_hoodMotor.setControl(m_hoodPositionRequest.withPosition(m_hoodTargetPosition));
    }
    @Override
    public void hoodAngle(Angle angle) {
        hoodAngle(angleToMechanismPosition(angle));
    }
    @Override
    public void hoodStop() {
        m_hoodMotor.disable();
    }

    @Override
    public void hoodZeroingDrive() {
        m_hoodMotor.setControl(m_hoodDutyCycle);
    }
    @Override
    public void hoodZero() {
        // PhoenixUtil.tryUntilOk(5, () -> m_hoodMotor.setPosition(hoodPositionHome));
        MotorAction.setMotorPosition("Shooter Hood", m_hoodMotor, hoodPositionHome).run();
    }
    @Override
    public boolean hoodCanZero() {
        return Math.abs(m_hoodVelocitySignal.getValueAsDouble()) <= hoodZeroVelocityThresholdRPS;
    }

    @Override
    public void upperKickerVelocity(AngularVelocity velocity) {
        m_upperKickerMotor.setControl(m_kickerVelocityRequest.withVelocity(velocity));
    }
    @Override
    public void upperKickerStop() {
        m_upperKickerMotor.disable();
    }
}
