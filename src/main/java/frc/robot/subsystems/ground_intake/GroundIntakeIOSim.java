package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.ground_intake.GroundIntakeConstants.*;
import static frc.robot.util.ConversionUtil.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public final class GroundIntakeIOSim implements GroundIntakeIO {
    // private double m_targetActuatorPositionAngle = distanceToMechanismPosition(actuatorPositionHome, actuatorMotorPitchCircumference);
    private Distance m_targetActuatorPosition = actuatorPositionHome;
    private Distance m_actuatorPosition = m_targetActuatorPosition;

    private double m_rollerVelocityRPS = 0d;

    private final PIDController m_actuatorPositionPID = new PIDController(0.5d, 0d, 0d);

    public GroundIntakeIOSim() {
        
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        m_actuatorPosition = m_actuatorPosition.plus(Inches.of(m_actuatorPositionPID.calculate(m_actuatorPosition.in(Inches))));

        final Distance targetActuatorPosition = m_targetActuatorPosition;
        
        inputs.isDeployed = targetActuatorPosition == actuatorPositionDeployed;

        final double targetRotations = distanceToMechanismPosition(targetActuatorPosition, actuatorMotorPitchCircumference);
        inputs.targetActuatorPositionRotations = targetRotations;
        inputs.targetActuatorPositionDegrees = mechanismPositionToAngle(targetRotations).in(Degrees);
        inputs.targetActuatorPositionInches = targetActuatorPosition.in(Inches);

        final double rotations = distanceToMechanismPosition(m_actuatorPosition, actuatorMotorPitchCircumference);
        inputs.actuatorPositionRotations = rotations;
        inputs.actuatorPositionDegrees = mechanismPositionToAngle(rotations).in(Degrees);
        inputs.actuatorPositionInches = m_actuatorPosition.in(Inches);

        inputs.rightRollerMotorVelocityRPS = m_rollerVelocityRPS;



    }

    @Override
    public void setActuatorPosition(Distance position) {
        m_targetActuatorPosition = position;
        m_actuatorPositionPID.setSetpoint(position.in(Inches));
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
