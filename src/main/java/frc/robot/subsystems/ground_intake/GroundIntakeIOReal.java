package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;
import static frc.robot.util.ConversionUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import frc.robot.state_machine.RobotEvent;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.PhoenixUtil.MotorAction;

public final class GroundIntakeIOReal implements GroundIntakeIO {
    private final TalonFX m_rightRollerMotor = new TalonFX(rightRollerMotorId, Constants.CANBUS);
    // private final TalonFX m_leftRollerMotor = new TalonFX(leftRollerMotorId, Constants.CANBUS);
    private final TalonFX m_actuatorMotor = new TalonFX(actuatorMotorId, Constants.CANBUS);

    private double m_actuatorTargetPosition = actuatorPositionHome;

    private final StatusSignal<AngularVelocity> m_rightRollerVelocitySignal = m_rightRollerMotor.getVelocity();
    private final StatusSignal<Current> m_rightRollerCurrentSignal = m_rightRollerMotor.getSupplyCurrent();

    // private final StatusSignal<AngularVelocity> m_leftRollerVelocitySignal = m_rightRollerMotor.getVelocity();
    // private final StatusSignal<Current> m_leftRollerCurrentSignal = m_rightRollerMotor.getSupplyCurrent();

    private final StatusSignal<Angle> m_actuatorPositionSignal = m_actuatorMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_actuatorVelocitySignal = m_actuatorMotor.getVelocity();
    private final StatusSignal<Current> m_actuatorCurrentSignal = m_actuatorMotor.getSupplyCurrent();

    private final MotionMagicVoltage m_actuatorPositionRequest = new MotionMagicVoltage(m_actuatorTargetPosition)
        .withEnableFOC(true);
    // private final DutyCycleOut m_rollerDutyCycleRequest = new DutyCycleOut(0d);
    private final MotionMagicVelocityVoltage m_rollerVelocityRequest = new MotionMagicVelocityVoltage(0d)
        .withEnableFOC(true);
    private final DutyCycleOut m_actuatorDutyCycle = new DutyCycleOut(actuatorZeroDutyCycle);

    public GroundIntakeIOReal() {
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = rollerNeutralMode;
        rollerConfig.MotorOutput.Inverted = rightRollerInverted;

        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = rollerCurrentLimit;

        // rollerConfig.Feedback.SensorToMechanismRatio = rollerMotorReduction;

        rollerConfig.Slot0.kP = rollerPidP;
        rollerConfig.Slot0.kV = rollerPidV;
        rollerConfig.MotionMagic.MotionMagicAcceleration = rollerTargetAcceleration;

        var actuatorConfig = new TalonFXConfiguration();
        actuatorConfig.MotorOutput.NeutralMode = actuatorNeutralMode;
        actuatorConfig.MotorOutput.Inverted = actuatorInverted;

        actuatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        actuatorConfig.CurrentLimits.SupplyCurrentLimit = actuatorCurrentLimit;

        actuatorConfig.Feedback.SensorToMechanismRatio = actuatorMotorReduction;

        actuatorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        actuatorConfig.Slot0.kP = actuatorPidP;

        actuatorConfig.MotionMagic.MotionMagicAcceleration = actuatorTargetAcceleration;
        actuatorConfig.MotionMagic.MotionMagicCruiseVelocity = actuatorMaxVelocity;

        // PhoenixUtil.tryUntilOk(5, () -> m_rollerMotor.getConfigurator().apply(rollerConfig));
        // PhoenixUtil.tryUntilOk(5, () -> m_actuatorMotor.getConfigurator().apply(actuatorConfig));
        MotorAction.configureMotor("Intake Roller", m_rightRollerMotor, rollerConfig).run();
        MotorAction.configureMotor("Intake Actuator", m_actuatorMotor, actuatorConfig).run();

        BaseStatusSignal.setUpdateFrequencyForAll(50d,
            m_rightRollerVelocitySignal,
            m_rightRollerCurrentSignal,
            // m_leftRollerVelocitySignal,
            // m_leftRollerCurrentSignal,
            m_actuatorPositionSignal,
            m_actuatorVelocitySignal,
            m_actuatorCurrentSignal);

        // PhoenixUtil.tryUntilOk(5, () -> m_actuatorMotor.setPosition(m_actuatorTargetPosition));
        MotorAction.setMotorPosition("Intake Acuator", m_actuatorMotor, m_actuatorTargetPosition).run();

        // m_leftRollerMotor.setControl(new Follower(rightRollerMotorId, leftRollerMotorAlignment));

        RobotEvent.OnTeleopEnabled.addListener(() -> setRollerCurrentLimit(rollerCurrentLimit));
        RobotEvent.OnAutoEnabled.addListener(() -> setRollerCurrentLimit(rollerCurrentLimitAuto));
    }
    
    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            m_rightRollerVelocitySignal,
            m_rightRollerCurrentSignal,
            // m_leftRollerVelocitySignal,
            // m_leftRollerCurrentSignal,
            m_actuatorPositionSignal,
            m_actuatorVelocitySignal,
            m_actuatorCurrentSignal);

        final double actuatorTargetPosition = m_actuatorTargetPosition;

        inputs.isDeployed = actuatorTargetPosition == actuatorPositionDeployed;

        inputs.targetActuatorPositionRotations = m_actuatorTargetPosition;
        inputs.targetActuatorPositionDegrees = mechanismPositionToAngle(m_actuatorTargetPosition).in(Degrees);

        final double actuatorPositionRotations = m_actuatorPositionSignal.getValueAsDouble();
        inputs.actuatorPositionRotations = actuatorPositionRotations;
        inputs.actuatorPositionDegrees = mechanismPositionToAngle(actuatorPositionRotations).in(Degrees);
        inputs.actuatorMotorCurrentAmps = m_actuatorCurrentSignal.getValueAsDouble();

        inputs.rightRollerMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_rightRollerMotor.getAppliedControl());
        inputs.rightRollerMotorVelocityRPS = m_rightRollerVelocitySignal.getValueAsDouble();
        inputs.rightRollerMotorCurrentAmps = m_rightRollerCurrentSignal.getValueAsDouble();

        // inputs.leftRollerMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_leftRollerMotor.getAppliedControl());
        // inputs.leftRollerMotorVelocityRPS = m_leftRollerVelocitySignal.getValueAsDouble();
        // inputs.leftRollerMotorCurrentAmps = m_leftRollerCurrentSignal.getValueAsDouble();
    }

    @Override
    public void idle() {
        m_rightRollerMotor.disable();
        m_actuatorMotor.disable();
    }

    @Override
    public void setActuatorPosition(double position) {
        m_actuatorTargetPosition = position;
        m_actuatorMotor.setControl(m_actuatorPositionRequest.withPosition(position));
    }
    @Override
    public void actuatorStop() {
        m_actuatorMotor.disable();
    }

    @Override
    public void rollerVelocity(AngularVelocity velocity) {
        m_rightRollerMotor.setControl(m_rollerVelocityRequest.withVelocity(velocity));
    }
    @Override
    public void rollerStop() {
        m_rightRollerMotor.disable();
    }

    @Override
    public void actuatorZeroingDrive() {
        m_actuatorMotor.setControl(m_actuatorDutyCycle);
    }
    @Override
    public void actuatorZero() {
        // PhoenixUtil.tryUntilOk(5, () -> m_actuatorMotor.setPosition(actuatorPositionHome));
        MotorAction.setMotorPosition("Shooter actuator", m_actuatorMotor, actuatorPositionHome).run();
    }
    @Override
    public boolean actuatorCanZero() {
        return Math.abs(m_actuatorVelocitySignal.getValueAsDouble()) <= actuatorZeroVelocityThresholdRPS;
    }

    private void setRollerCurrentLimit(double limit) {
        MotorAction.updateMotorConfig("Intake Roller", m_rightRollerMotor, (config) -> {
            config.CurrentLimits.SupplyCurrentLimit = limit;
        }).queue();
    }
}
