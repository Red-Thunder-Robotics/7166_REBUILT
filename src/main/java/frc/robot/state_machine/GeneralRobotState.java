package frc.robot.state_machine;

public enum GeneralRobotState {
    Idle,
    HubTracking,
    HubTrackingFiring,
    AllianceFeed,
    AllianceFeedFiring,
    DemoMode,
    DemoModeFiring;

    public boolean isFiring() {
        switch (this) {
            case Idle:
            case HubTracking:
            case AllianceFeed:
            case DemoMode:
                return false;
            case HubTrackingFiring:
            case AllianceFeedFiring:
            case DemoModeFiring:
                return true;
        }

        return false;
    }
}
