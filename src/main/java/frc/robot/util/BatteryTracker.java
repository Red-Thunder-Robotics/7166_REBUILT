package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Tracks which battery is in the robot and how much it has been used since the robot turned on.
 *
 * <p>Our PDP 2.0 is not on CAN, so total current can't be read from it. Instead this adds up the supply current
 * reported by each motor controller, plus the roboRIO's own current. The radio, CANivore, Limelights and LEDs are not
 * measured, so the totals read a little low.
 *
 * <p>Swapping a battery reboots the robot, which resets the chooser to "Not selected" and the totals to zero, so each
 * log file covers one battery.
 */
public final class BatteryTracker {
    private static final String NOT_SELECTED = "Not selected";

    /** Names on the battery labels. Edit this list when batteries are added or retired. */
    private static final String[] BATTERY_NAMES = {
        "RTR-01", "RTR-02", "RTR-03", "RTR-04", "RTR-05", "RTR-06", "RTR-07", "RTR-08", "RTR-09", "RTR-10",
    };

    private final LoggedDashboardChooser<String> m_chooser = new LoggedDashboardChooser<>("Battery");
    private final List<DoubleSupplier> m_currentSources;
    private final Alert m_notSelectedAlert =
            new Alert("No battery selected. Pick one in the Battery dropdown.", AlertType.kWarning);

    private double m_lastTimestamp = -1d;
    private double m_ampHours = 0d;
    private double m_wattHours = 0d;

    /** @param currentSources each returns the current one subsystem draws from the battery, in amps */
    public BatteryTracker(List<DoubleSupplier> currentSources) {
        m_currentSources = currentSources;

        m_chooser.addDefaultOption(NOT_SELECTED, NOT_SELECTED);
        for (final String name : BATTERY_NAMES) m_chooser.addOption(name, name);
    }

    public void periodic() {
        double currentAmps = RobotController.getInputCurrent();
        for (final var source : m_currentSources) currentAmps += source.getAsDouble();
        final double voltage = RobotController.getBatteryVoltage();

        // Add up current x time (amp-hours) and power x time (watt-hours) since the last loop
        final double now = Timer.getTimestamp();
        if (m_lastTimestamp >= 0d) {
            final double dtHours = (now - m_lastTimestamp) / 3600d;
            m_ampHours += currentAmps * dtHours;
            m_wattHours += currentAmps * voltage * dtHours;
        }
        m_lastTimestamp = now;

        final String batteryName = m_chooser.get();
        m_notSelectedAlert.set(NOT_SELECTED.equals(batteryName));

        Logger.recordOutput("Battery/Name", batteryName);
        Logger.recordOutput("Battery/VoltageVolts", voltage);
        Logger.recordOutput("Battery/TotalCurrentAmps", currentAmps);
        Logger.recordOutput("Battery/AmpHoursUsed", m_ampHours);
        Logger.recordOutput("Battery/WattHoursUsed", m_wattHours);
    }
}
