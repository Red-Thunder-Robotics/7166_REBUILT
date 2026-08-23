package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machine.ClimberState;
import org.littletonrobotics.junction.Logger;

public final class ClimberSubsystem extends SubsystemBase {
    public static ClimberSubsystem instance = null;

    private final ClimberIO m_io;
    private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();

    public ClimberSubsystem(ClimberIO io) {
        instance = this;

        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Climber", m_inputs);
        Logger.recordOutput("Climber/PositionDeployedRotations", positionDeployed);
    }

    public void stateUpdate(ClimberState climberState) {
        switch (climberState) {
            case Idle:
                m_io.stop();
                break;
            case Home:
                m_io.setPosition(positionHome);
                break;
            case Deployed:
                m_io.setPosition(positionDeployed);
                break;
        }
    }
}
