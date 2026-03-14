package frc.robot.subsystems.climbermark1;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.climbermark1.ClimberConstants.*;
import static frc.robot.util.ConversionUtil.mechanismPositionToDistance;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public final class ClimberIOReal implements ClimberIO {
    // private final TalonFX m_leftMotor = new TalonFX(leftMotorId, Constants.CANBUS);
    private final TalonFX m_rightMotor = new TalonFX(rightMotorId, Constants.CANBUS);

    private double m_leftTargetPosition = positionHome;
    private double m_rightTargetPosition = positionHome;

    // private final StatusSignal<Angle> m_leftPositionSignal = m_leftMotor.getPosition();
    // private final StatusSignal<AngularVelocity> m_leftVelocitySignal = m_leftMotor.getVelocity();
    // private final StatusSignal<Current> m_leftCurrentSignal = m_leftMotor.getSupplyCurrent();

    private final StatusSignal<Angle> m_rightPositionSignal = m_rightMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_rightVelocitySignal = m_rightMotor.getVelocity();
    private final StatusSignal<Current> m_rightCurrentSignal = m_rightMotor.getSupplyCurrent();
    
    private final MotionMagicVoltage m_leftPositionRequest = new MotionMagicVoltage(m_leftTargetPosition);
    private final MotionMagicVoltage m_rightPositionRequest = new MotionMagicVoltage(m_rightTargetPosition);

    public ClimberIOReal() {
        // var leftConfig = new TalonFXConfiguration();
        // leftConfig.MotorOutput.NeutralMode = neutralMode;
        // leftConfig.MotorOutput.Inverted = leftInverted;

        // leftConfig.Feedback.SensorToMechanismRatio = motorReduction;

        // leftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        // leftConfig.Slot0.kP = pidP;

        // leftConfig.MotionMagic.MotionMagicAcceleration = targetAcceleration;
        // leftConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;

        // var rightConfig = leftConfig.clone();
        // rightConfig.MotorOutput.Inverted = rightInverted;

        var rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = neutralMode;
        rightConfig.MotorOutput.Inverted = rightInverted;

        rightConfig.Feedback.SensorToMechanismRatio = motorReduction;

        rightConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        rightConfig.Slot0.kP = pidP;

        rightConfig.MotionMagic.MotionMagicAcceleration = targetAcceleration;
        rightConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;

        // PhoenixUtil.tryUntilOk(5, () -> m_leftMotor.getConfigurator().apply(leftConfig));
        PhoenixUtil.tryUntilOk(5, () -> m_rightMotor.getConfigurator().apply(rightConfig));

        // BaseStatusSignal.setUpdateFrequencyForAll(50d, m_leftPositionSignal, m_leftVelocitySignal, m_leftCurrentSignal, m_rightPositionSignal, m_rightVelocitySignal, m_rightCurrentSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(50d,  m_rightPositionSignal, m_rightVelocitySignal, m_rightCurrentSignal);

        // PhoenixUtil.tryUntilOk(5, () -> m_leftMotor.setPosition(m_leftTargetPosition));
        PhoenixUtil.tryUntilOk(5, () -> m_rightMotor.setPosition(m_rightTargetPosition));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // BaseStatusSignal.refreshAll(m_leftPositionSignal, m_leftVelocitySignal, m_leftCurrentSignal, m_rightPositionSignal, m_rightVelocitySignal, m_rightCurrentSignal);
        BaseStatusSignal.refreshAll(m_rightPositionSignal, m_rightVelocitySignal, m_rightCurrentSignal);

        // left
        final double leftTargetPosition = m_leftTargetPosition;
        inputs.leftTargetPositionRotations = leftTargetPosition;
        inputs.leftTargetPositionInches = mechanismPositionToDistance(leftTargetPosition, pitchCircumference).in(Inches);

        // final double leftPositionRotations = m_leftPositionSignal.getValueAsDouble();
        // inputs.leftPositionRotations = leftPositionRotations;
        // inputs.leftPositionInches = mechanismPositionToDistance(leftPositionRotations, pitchCircumference).in(Inches);

        // inputs.leftVelocityRPS = m_leftVelocitySignal.getValueAsDouble();
        // inputs.leftCurrentAmps = m_leftCurrentSignal.getValueAsDouble();

        // right
        final double rightTargetPosition = m_rightTargetPosition;
        inputs.rightTargetPositionRotations = rightTargetPosition;
        inputs.rightTargetPositionInches = mechanismPositionToDistance(rightTargetPosition, pitchCircumference).in(Inches);

        final double rightPositionRotations = m_rightPositionSignal.getValueAsDouble();
        inputs.rightPositionRotations = rightPositionRotations;
        inputs.rightPositionInches = mechanismPositionToDistance(rightPositionRotations, pitchCircumference).in(Inches);

        inputs.rightVelocityRPS = m_rightVelocitySignal.getValueAsDouble();
        inputs.rightCurrentAmps = m_rightCurrentSignal.getValueAsDouble();
    }

    @Override
    public void leftIdle() {
        // m_leftMotor.disable();
    }
    @Override
    public void rightIdle() {
        m_rightMotor.disable();
    }

    @Override
    public void setLeftPosition(double position) {
        m_leftTargetPosition = position;
        // m_leftMotor.setControl(m_leftPositionRequest.withPosition(position));
    }
    @Override
    public void setRightPosition(double position) {
        m_rightTargetPosition = position;
        m_rightMotor.setControl(m_rightPositionRequest.withPosition(position));
    }
}
