package frc.robot.subsystems.light_emitting_diodes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machine.GeneralRobotState;

public final class LightEmittingDiodesSubsystem extends SubsystemBase {
    public static LightEmittingDiodesSubsystem instance = null;

    private final LightEmittingDiodesIO m_io;

    private GeneralRobotState lastGeneralRobotState = null;

    public LightEmittingDiodesSubsystem(LightEmittingDiodesIO io) {
        instance = this;

        m_io = io;
    }

    public void stateUpdate(GeneralRobotState generalRobotState) {
        if (lastGeneralRobotState != null && generalRobotState == lastGeneralRobotState) return;

        lastGeneralRobotState = generalRobotState;
        switch (generalRobotState) {
            case Idle:
                m_io.solidWhite();
                break;
            case HubTracking:
                m_io.solidGreen();
                break;
            case HubTrackingFiring:
                m_io.flashingGreen();
                break;
            case AllianceFeed:
                m_io.solidBlue();
                break;
            case AllianceFeedFiring:
                m_io.flashingBlue();
                break;
        }
    }
}
