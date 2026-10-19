package frc.robot.sim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.util.BeachedUtil;

/**
 * Runs one PathPlanner auto in simulation without a driver station, then exits.
 *
 * Set these environment variables before `./gradlew simulateJava`:
 * SIM_AUTO (auto file name), SIM_LOG_DIR (folder for the .wpilog),
 * SIM_ALLIANCE (Blue or Red, default Blue), SIM_SECONDS (auto length, default 20),
 * SIM_BEACHED=0 to turn off BeachedUtil, SIM_FUEL_CLUMP=0 to remove the fuel clump.
 * When SIM_AUTO is unset this class does nothing.
 */
public final class SimAutoRunner {
    private static final String autoName = System.getenv("SIM_AUTO");
    private static final double startDelaySeconds = 2d;
    private static final double autoSeconds = parseDouble(System.getenv("SIM_SECONDS"), 20d);

    private static final Timer timer = new Timer();
    private static boolean configured = false;

    private SimAutoRunner() {}

    public static boolean isActive() {
        return autoName != null && !autoName.isBlank();
    }

    public static String getAutoName() {
        return autoName;
    }

    public static String getLogDir() {
        String dir = System.getenv("SIM_LOG_DIR");
        return dir == null || dir.isBlank() ? "logs" : dir;
    }

    /** Call from simulationPeriodic. Disabled for a moment, then auto, then exit. */
    public static void periodic() {
        if (!isActive())
            return;

        if (!configured) {
            configured = true;
            final boolean beachedOff = "0".equals(System.getenv("SIM_BEACHED"));
            BeachedUtil.getInstance().setIgnoreBeached(() -> beachedOff);
            boolean red = "Red".equalsIgnoreCase(System.getenv("SIM_ALLIANCE"));
            DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setAutonomous(true);
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            timer.restart();
        }

        final double t = timer.get();
        if (t >= startDelaySeconds && t < startDelaySeconds + autoSeconds && !DriverStationSim.getEnabled()) {
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
        } else if (t >= startDelaySeconds + autoSeconds && DriverStationSim.getEnabled()) {
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
        } else if (t >= startDelaySeconds + autoSeconds + 1d) {
            Logger.end();
            System.exit(0);
        }
    }

    private static double parseDouble(String value, double fallback) {
        try {
            return value == null ? fallback : Double.parseDouble(value);
        } catch (NumberFormatException e) {
            return fallback;
        }
    }
}
