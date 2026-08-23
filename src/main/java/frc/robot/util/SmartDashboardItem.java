package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

public final class SmartDashboardItem {
    private static final ArrayList<Item> periodicList = new ArrayList<>();
    private static final ArrayList<Item> disabledList = new ArrayList<>();

    public static void processAll() {
        periodicList.forEach(Item::poll);
        disabledList.forEach(Item::poll);
    }

    public static void processPeriodic() {
        periodicList.forEach(Item::poll);
    }

    public static void newButtonPeriodic(String name, Runnable callback) {
        periodicList.add(new Button(name, callback));
    }

    public static void newButtonDisabled(String name, Runnable callback) {
        disabledList.add(new Button(name, callback));
    }

    public static void newTogglePeriodic(String name, BooleanConsumer callback) {
        periodicList.add(new Toggle(name, callback));
    }

    public static void newToggleDisabled(String name, BooleanConsumer callback) {
        disabledList.add(new Toggle(name, callback));
    }

    private interface Item {
        void poll();
    }

    private static final class Button implements Item {
        private final NetworkTableEntry m_entry;
        private final Runnable m_callback;

        private boolean m_lastValue;

        public Button(String name, Runnable callback) {
            m_entry = SmartDashboard.getEntry(name);
            m_callback = callback;

            m_entry.setBoolean(false);
        }

        @Override
        public void poll() {
            final boolean value = m_entry.getBoolean(m_lastValue);
            if (value && !m_lastValue) m_callback.run();
            m_lastValue = value;
        }
    }

    private static final class Toggle implements Item {
        private final NetworkTableEntry m_entry;
        private final BooleanConsumer m_callback;

        private boolean m_lastValue;

        public Toggle(String name, BooleanConsumer callback) {
            m_entry = SmartDashboard.getEntry(name);
            m_callback = callback;

            m_entry.setBoolean(false);
        }

        @Override
        public void poll() {
            final boolean value = m_entry.getBoolean(m_lastValue);
            if (value != m_lastValue) m_callback.accept(value);
            m_lastValue = value;
        }
    }
}
