package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.climber.ClimberConstants.*;
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
    private final TalonFX m_rightMotor = new TalonFX(rightMotorId, Constants.CANBUS);

    private double m_rightTargetPosition = positionHome;

    private final StatusSignal<Angle> m_rightPositionSignal = m_rightMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_rightVelocitySignal = m_rightMotor.getVelocity();
    private final StatusSignal<Current> m_rightCurrentSignal = m_rightMotor.getSupplyCurrent();
    
    private final MotionMagicVoltage m_rightPositionRequest = new MotionMagicVoltage(m_rightTargetPosition);

    public ClimberIOReal() {
        var rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = neutralMode;
        rightConfig.MotorOutput.Inverted = rightInverted;

        rightConfig.Feedback.SensorToMechanismRatio = motorReduction;

        rightConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        rightConfig.Slot0.kP = pidP;

        rightConfig.MotionMagic.MotionMagicAcceleration = targetAcceleration;
        rightConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;

        PhoenixUtil.tryUntilOk(5, () -> m_rightMotor.getConfigurator().apply(rightConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d,  m_rightPositionSignal, m_rightVelocitySignal, m_rightCurrentSignal);

        PhoenixUtil.tryUntilOk(5, () -> m_rightMotor.setPosition(m_rightTargetPosition));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(m_rightPositionSignal, m_rightVelocitySignal, m_rightCurrentSignal);

        // right
        final double rightTargetPosition = m_rightTargetPosition;
        inputs.positionRotations = rightTargetPosition;
        inputs.positionInches = mechanismPositionToDistance(rightTargetPosition, pitchCircumference).in(Inches);

        final double rightPositionRotations = m_rightPositionSignal.getValueAsDouble();
        inputs.positionRotations = rightPositionRotations;
        inputs.positionInches = mechanismPositionToDistance(rightPositionRotations, pitchCircumference).in(Inches);

        inputs.velocityRPS = m_rightVelocitySignal.getValueAsDouble();
        inputs.currentAmps = m_rightCurrentSignal.getValueAsDouble();
    }

    @Override
    public void setPosition(double position) {
        m_rightTargetPosition = position;
        m_rightMotor.setControl(m_rightPositionRequest.withPosition(position));
    }
    @Override
    public void stop() {
        m_rightMotor.disable();
    }
}
