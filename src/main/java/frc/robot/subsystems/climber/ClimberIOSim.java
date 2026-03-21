package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.ConversionUtil.mechanismPositionToDistance;

import edu.wpi.first.math.controller.PIDController;

public final class ClimberIOSim implements ClimberIO {
    private double m_leftTargetPosition = positionHome;
    private double m_leftPosition = m_leftTargetPosition;

    private double m_rightTargetPosition = positionHome;
    private double m_rightPosition = m_rightTargetPosition;

    private final PIDController m_leftPID = new PIDController(0.5d, 0d, 0d);
    private final PIDController m_rightPID = new PIDController(0.5d, 0d, 0d);

    public ClimberIOSim() {

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // left
        final double leftOutput = m_leftPID.calculate(m_leftPosition);
        m_leftPosition += leftOutput;

        final double leftTargetPosition = m_leftTargetPosition;
        inputs.leftTargetPositionRotations = leftTargetPosition;
        inputs.leftTargetPositionInches = mechanismPositionToDistance(leftTargetPosition, pitchCircumference).in(Inches);

        final double leftPosition = m_leftPosition;
        inputs.leftPositionRotations = leftPosition;
        inputs.leftPositionInches = mechanismPositionToDistance(leftPosition, pitchCircumference).in(Inches);

        inputs.leftDutyCycle = leftOutput;

        // right
        final double rightOutput = m_rightPID.calculate(m_rightPosition);
        m_rightPosition += rightOutput;

        final double rightTargetPosition = m_rightTargetPosition;
        inputs.rightTargetPositionRotations = rightTargetPosition;
        inputs.rightTargetPositionInches = mechanismPositionToDistance(rightTargetPosition, pitchCircumference).in(Inches);

        final double rightPosition = m_rightPosition;
        inputs.rightPositionRotations = rightPosition;
        inputs.rightPositionInches = mechanismPositionToDistance(rightPosition, pitchCircumference).in(Inches);

        inputs.rightDutyCycle = rightOutput;
    }

    @Override
    public void setLeftPosition(double position) {
        m_leftTargetPosition = position;
        m_leftPID.setSetpoint(position);
    }
    @Override
    public void setRightPosition(double position) {
        m_rightTargetPosition = position;
        m_rightPID.setSetpoint(position);
    }
}
