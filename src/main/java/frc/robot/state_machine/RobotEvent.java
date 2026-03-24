package frc.robot.state_machine;

import java.util.ArrayList;

public enum RobotEvent {
    TurboOn,
    TurboOff,
    OnEnabled,
    OnDisabled,
    OnTeleopEnabled,
    OnAutoEnabled,
    OnShootingStart,
    OnShootingEnd,
    ;

    private final ArrayList<Runnable> list = new ArrayList<>();

    public void addListener(Runnable listener) {
        list.add(listener);
    }

    public void trigger() {
        list.forEach(Runnable::run);
    }
}
