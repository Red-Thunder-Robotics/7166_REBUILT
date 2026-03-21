package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.ConversionUtil.mechanismPositionToDistance;

import edu.wpi.first.math.controller.PIDController;

public final class ClimberIOSim implements ClimberIO {
    private double m_targetPosition = positionHome;
    private double m_position = m_targetPosition;

    private final PIDController m_PID = new PIDController(0.5d, 0d, 0d);

    public ClimberIOSim() {

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // right
        final double output = m_PID.calculate(m_position);
        m_position += output;

        final double targetPosition = m_targetPosition;
        inputs.positionRotations = targetPosition;
        inputs.positionInches = mechanismPositionToDistance(targetPosition, pitchCircumference).in(Inches);

        final double position = m_position;
        inputs.positionRotations = position;
        inputs.positionInches = mechanismPositionToDistance(position, pitchCircumference).in(Inches);
    }

    @Override
    public void setPosition(double position) {
        m_targetPosition = position;
        m_PID.setSetpoint(position);
    }
}
