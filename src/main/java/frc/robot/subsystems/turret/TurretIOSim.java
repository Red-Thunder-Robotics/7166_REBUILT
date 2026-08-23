package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import edu.wpi.first.math.controller.PIDController;

public final class TurretIOSim implements TurretIO {
    private double m_targetPosition = positionHome;
    private double m_position = m_targetPosition;

    private final PIDController m_pid = new PIDController(0.5d, 0d, 0d);

    public TurretIOSim() {}

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        final double output = m_pid.calculate(m_position);
        m_position += output;

        final double targetPosition = m_targetPosition;

        inputs.targetPositionRotations = targetPosition;
        inputs.targetPositionDegrees = mechanismPositionToAngle(targetPosition).in(Degrees);

        final double position = m_position;
        inputs.positionRotations = position;
        inputs.positionDegrees = mechanismPositionToAngle(position).in(Degrees);

        inputs.dutyCycle = output;
    }

    @Override
    public void setPosition(double position) {
        if (position < positionMin) position = positionMin;
        else if (position > positionMax) position = positionMax;

        m_targetPosition = position;
        m_pid.setSetpoint(position);
    }
}
