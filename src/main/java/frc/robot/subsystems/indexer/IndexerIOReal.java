package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.PhoenixUtil.MotorAction;

public final class IndexerIOReal implements IndexerIO {
    private final TalonFX m_indexerMotor = new TalonFX(indexerMotorId, Constants.CANBUS);
    private final TalonFX m_topRollerMotor = new TalonFX(topRollerMotorId, Constants.CANBUS);
    private final TalonFX m_lowerKickerMotor = new TalonFX(lowerKickerMotorId, Constants.CANBUS);

    private final StatusSignal<AngularVelocity> m_indexerVelocitySignal = m_indexerMotor.getVelocity();
    private final StatusSignal<Current> m_indexerCurrentSignal = m_indexerMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_topRollerVelocitySignal = m_topRollerMotor.getVelocity();
    private final StatusSignal<Current> m_topRollerCurrentSignal = m_topRollerMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_lowerKickerVelocitySignal = m_lowerKickerMotor.getVelocity();
    private final StatusSignal<Current> m_lowerKickerCurrentSignal = m_lowerKickerMotor.getSupplyCurrent();

    private final MotionMagicVelocityVoltage m_indexerVelocityRequest = new MotionMagicVelocityVoltage(0d)
        .withEnableFOC(true);
        // .withEnableFOC(false);
    private final MotionMagicVelocityVoltage m_topRollerVelocityRequest = new MotionMagicVelocityVoltage(0d)
        .withEnableFOC(true);
    private final MotionMagicVelocityVoltage m_lowerKickerVelocityRequest = new MotionMagicVelocityVoltage(0d)
        .withEnableFOC(true);

    public IndexerIOReal() {
        var indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = indexerNeutralMode;
        indexerConfig.MotorOutput.Inverted = indexerInverted;
        //
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = indexerCurrentLimit;
        //
        indexerConfig.Feedback.SensorToMechanismRatio = indexerMotorReduction;
        //
        indexerConfig.Slot0.kP = indexerPidP;
        indexerConfig.Slot0.kV = indexerPidV;
        indexerConfig.MotionMagic.MotionMagicAcceleration = indexerTargetAcceleration;

        var topRollerConfig = new TalonFXConfiguration();
        topRollerConfig.MotorOutput.NeutralMode = topRollerNeutralMode;
        topRollerConfig.MotorOutput.Inverted = topRollerInverted;
        //
        topRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        topRollerConfig.CurrentLimits.SupplyCurrentLimit = topRollerCurrentLimit;
        //
        topRollerConfig.Feedback.SensorToMechanismRatio = topRollerReduction;
        //
        topRollerConfig.Slot0.kP = topRollerPidP;
        topRollerConfig.Slot0.kV = topRollerPidV;
        topRollerConfig.MotionMagic.MotionMagicAcceleration = topRollerTargetAcceleration;

        var lowerKickerConfig = new TalonFXConfiguration();
        lowerKickerConfig.MotorOutput.NeutralMode = lowerKickerNeutralMode;
        lowerKickerConfig.MotorOutput.Inverted = lowerKickerInverted;
        //
        lowerKickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        lowerKickerConfig.CurrentLimits.SupplyCurrentLimit = lowerKickerCurrentLimit;
        //
        lowerKickerConfig.Slot0.kP = indexerPidP;
        lowerKickerConfig.Slot0.kV = indexerPidV;
        lowerKickerConfig.MotionMagic.MotionMagicAcceleration = indexerTargetAcceleration;

        // PhoenixUtil.tryUntilOk(5, () -> m_indexerMotor.getConfigurator().apply(indexerConfig));
        // PhoenixUtil.tryUntilOk(5, () -> m_topRollerMotor.getConfigurator().apply(topRollerConfig));
        // PhoenixUtil.tryUntilOk(5, () -> m_lowerKickerMotor.getConfigurator().apply(lowerKickerConfig));
        MotorAction.configureMotor("Indexer Roller", m_indexerMotor, indexerConfig).run();
        MotorAction.configureMotor("Indexer Top Roller", m_topRollerMotor, topRollerConfig).run();
        MotorAction.configureMotor("Lower Kicker", m_lowerKickerMotor, lowerKickerConfig).run();

        BaseStatusSignal.setUpdateFrequencyForAll(50d,
            m_indexerVelocitySignal,
            m_indexerCurrentSignal,
            m_topRollerVelocitySignal,
            m_topRollerCurrentSignal,
            m_lowerKickerVelocitySignal,
            m_lowerKickerCurrentSignal
        );
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            m_indexerVelocitySignal,
            m_indexerCurrentSignal,
            m_topRollerVelocitySignal,
            m_topRollerCurrentSignal,
            m_lowerKickerVelocitySignal,
            m_lowerKickerCurrentSignal
        );

        inputs.indexerTargetVelocityRPS = PhoenixUtil.getRequestVelocity(m_indexerMotor.getAppliedControl());
        inputs.indexerVelocityRPS = m_indexerVelocitySignal.getValueAsDouble();
        inputs.indexerCurrentAmps = m_indexerCurrentSignal.getValueAsDouble();

        inputs.topRollerTargetVelocityRPS = PhoenixUtil.getRequestVelocity(m_topRollerMotor.getAppliedControl());
        inputs.topRollerVelocityRPS = m_topRollerVelocitySignal.getValueAsDouble();
        inputs.topRollerCurrentAmps = m_topRollerCurrentSignal.getValueAsDouble();

        inputs.lowerKickerTargetVelocityRPS = PhoenixUtil.getRequestVelocity(m_lowerKickerMotor.getAppliedControl());
        inputs.lowerKickerVelocityRPS = m_lowerKickerVelocitySignal.getValueAsDouble();
        inputs.lowerKickerCurrentAmps = m_lowerKickerCurrentSignal.getValueAsDouble();
    }

    @Override
    public void indexerVelocity(AngularVelocity velocity) {
        m_indexerMotor.setControl(m_indexerVelocityRequest.withVelocity(velocity));
    }
    @Override
    public void indexerStop() {
        m_indexerMotor.disable();
    }

    @Override
    public void topRollerVelocity(AngularVelocity velocity) {
        m_topRollerMotor.setControl(m_topRollerVelocityRequest.withVelocity(velocity));
    }
    @Override
    public void topRollerStop() {
        m_topRollerMotor.disable();
    }

    @Override
    public void lowerKickerVelocity(AngularVelocity velocity) {
        m_lowerKickerMotor.setControl(m_lowerKickerVelocityRequest.withVelocity(velocity));
    }
    @Override
    public void lowerKickerStop() {
        m_lowerKickerMotor.disable();
    }
}
