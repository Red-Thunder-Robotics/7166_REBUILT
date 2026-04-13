package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;

public final class GroundIntakeIOSim implements GroundIntakeIO {
    private double m_targetActuatorPosition = actuatorPositionHome;
    private double m_actuatorPosition = m_targetActuatorPosition;

    private double m_rollerVelocityRPS = 0d;

    private final PIDController m_actuatorPositionPID = new PIDController(0.5d, 0d, 0d);

    public GroundIntakeIOSim() {
        
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        m_actuatorPosition += m_actuatorPositionPID.calculate(m_actuatorPosition);

        final double targetActuatorPosition = m_targetActuatorPosition;
        
        inputs.targetActuatorPositionRotations = targetActuatorPosition;
        inputs.targetActuatorPositionDegrees = mechanismPositionToAngle(targetActuatorPosition).in(Degrees);

        final double position = m_actuatorPosition;
        inputs.actuatorPositionRotations = position;
        inputs.actuatorPositionDegrees = mechanismPositionToAngle(position).in(Degrees);

        inputs.rightRollerMotorVelocityRPS = m_rollerVelocityRPS;
    }

    @Override
    public void setActuatorPosition(double position) {
        m_targetActuatorPosition = position;
        m_actuatorPositionPID.setSetpoint(position);
    }
    @Override
    public void rollerVelocity(AngularVelocity velocity) {
        m_rollerVelocityRPS = velocity.in(RotationsPerSecond);
    }
    @Override
    public void rollerStop() {
        m_rollerVelocityRPS = 0d;
    }
}
