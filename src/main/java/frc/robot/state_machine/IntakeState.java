package frc.robot.state_machine;

public enum IntakeState {
    HomeOff,
    HomeReverse,
    DeployedOff,
    DeployedOn,
    DeployedReverse,
    OscillateOff,
    OscillateOn;

    public boolean areRollersPowered() {
        switch (this) {
            case HomeReverse:
            case DeployedOn:
            case DeployedReverse:
            case OscillateOn:
                return true;
            case HomeOff:
            case DeployedOff:
            case OscillateOff:
                return false;
        }
        return false;
    }
    public boolean areRollersReversed() {
        switch (this) {
            case DeployedReverse:
            case HomeReverse:
                return true;

            default:
                return false;
        }
    }
    public boolean isDeployed() {
        switch (this) {
            case DeployedOff:
            case DeployedOn:
            case DeployedReverse:
                return true;

            default:
                return false;
        }
    }
    public boolean isOscillate() {
        switch (this) {
            case OscillateOff:
            case OscillateOn:
                return true;

            default:
                return false;
        }
    }
    public boolean isOut() {
        switch (this) {
            case DeployedOff:
            case DeployedOn:
            case DeployedReverse:
            case OscillateOff:
            case OscillateOn:
                return true;

            default:
                return false;
        }
    }
    public IntakeState off() {
        switch (this) {
            case DeployedOff:
            case DeployedOn:
            case DeployedReverse:
                return DeployedOff;
            case HomeReverse:
                return HomeOff;
            case OscillateOn:
                return OscillateOff;
            default:
                return this;
        }
    }
    public IntakeState on() {
        switch (this) {
            case DeployedOff:
            case DeployedOn:
            case DeployedReverse:
                return DeployedOn;
            case OscillateOff:
                return OscillateOn;
            default:
                return this;
        }
    }
    public IntakeState reverse() {
        switch (this) {
            case DeployedOff:
            case DeployedOn:
            case DeployedReverse:
                return DeployedReverse;
            case HomeOff:
                return HomeReverse;
            default:
                return this;
        }
    }
}
