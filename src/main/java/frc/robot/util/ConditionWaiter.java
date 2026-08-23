package frc.robot.util;

import java.util.function.BooleanSupplier;

public final class ConditionWaiter {
    private boolean m_active = false;
    private final BooleanSupplier m_condition;

    public ConditionWaiter(BooleanSupplier condition) {
        m_condition = condition;
    }

    public void activate() {
        m_active = true;
    }

    public void deactivate() {
        m_active = false;
    }

    public boolean check() {
        return m_active && m_condition.getAsBoolean();
    }

    public boolean process() {
        if (check()) {
            deactivate();
            return true;
        }
        return false;
    }
}
