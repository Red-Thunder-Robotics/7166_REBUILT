package frc.robot.state_machine;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class LiveConfig {
    private static final NetworkTable networkTable = NetworkTableInstance.getDefault()
        .getTable("Config");

    private static final BooleanTopic isPitTopic = networkTable.getBooleanTopic("IsPit");
    private static final BooleanSubscriber isPitSubscriber = isPitTopic.subscribe(false);

    private static final BooleanTopic visionFailTopic = networkTable.getBooleanTopic("VisionFail");
    private static final BooleanSubscriber visionFailSubscriber = visionFailTopic.subscribe(false);

    private static final BooleanTopic launcherTuning = networkTable.getBooleanTopic("LauncherTuning");
    private static final BooleanSubscriber launcherTuningSubscriber = launcherTuning.subscribe(false);

    private static final DoubleTopic launcherTuningRPM = networkTable.getDoubleTopic("LauncherTuningRPM");
    private static final DoubleSubscriber launcherTuningRPMSubscriber = launcherTuningRPM.subscribe(0d);

    private static final DoubleTopic launcherTuningAngle = networkTable.getDoubleTopic("LauncherTuningAngle");
    private static final DoubleSubscriber launcherTuningAngleSubscriber = launcherTuningAngle.subscribe(0d);

    static {
        isPitTopic.publish().set(false);
        visionFailTopic.publish().set(false);
        launcherTuning.publish().set(false);
        launcherTuningRPM.publish().set(launcherTuningRPMSubscriber.get());
        launcherTuningAngle.publish().set(launcherTuningAngleSubscriber.get());
    }

    public static boolean getIsPit() {
        return isPitSubscriber.get();
    }

    public static boolean getVisionFail() {
        return visionFailSubscriber.get();
    }

    public static boolean getLauncherTuning() {
        return launcherTuningSubscriber.get();
    }

    public static double getLauncherTuningRPM() {
        return launcherTuningRPMSubscriber.get();
    }

    public static double getLauncherTuningAngle() {
        return launcherTuningAngleSubscriber.get();
    }
}
