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
import frc.robot.util.PhoenixUtil.MotorAction;

public final class ClimberIOReal implements ClimberIO {
    private final TalonFX m_motor = new TalonFX(motorId, Constants.CANBUS);

    private double m_targetPosition = positionHome;

    private final StatusSignal<Angle> m_positionSignal = m_motor.getPosition();
    private final StatusSignal<AngularVelocity> m_velocitySignal = m_motor.getVelocity();
    private final StatusSignal<Current> m_currentSignal = m_motor.getSupplyCurrent();

    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(m_targetPosition);

    public ClimberIOReal() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = inverted;

        config.Feedback.SensorToMechanismRatio = motorReduction;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kP = pidP;

        config.MotionMagic.MotionMagicAcceleration = targetAcceleration;
        config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;

        // PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));
        MotorAction.configureMotor("Climber", m_motor, config).run();

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_positionSignal, m_velocitySignal, m_currentSignal);

        // PhoenixUtil.tryUntilOk(5, () -> m_rightMotor.setPosition(m_targetPosition));
        MotorAction.setMotorPosition("Climber", m_motor, m_targetPosition).run();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(m_positionSignal, m_velocitySignal, m_currentSignal);

        final double targetPosition = m_targetPosition;
        inputs.positionRotations = targetPosition;
        inputs.positionInches =
                mechanismPositionToDistance(targetPosition, pitchCircumference).in(Inches);

        final double positionRotations = m_positionSignal.getValueAsDouble();
        inputs.positionRotations = positionRotations;
        inputs.positionInches = mechanismPositionToDistance(positionRotations, pitchCircumference)
                .in(Inches);

        inputs.velocityRPS = m_velocitySignal.getValueAsDouble();
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
    }

    @Override
    public void setPosition(double position) {
        m_targetPosition = position;
        m_motor.setControl(m_positionRequest.withPosition(position));
    }

    @Override
    public void stop() {
        m_motor.disable();
    }
}
