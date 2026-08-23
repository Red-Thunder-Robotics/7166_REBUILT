package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;

public class VisionIOMackinac implements VisionIO {
    private final AprilTagFieldLayout m_fieldLayout;

    private static final NetworkTable cameraPublisherTable =
            NetworkTableInstance.getDefault().getTable("CameraPublisher");

    private final String m_deviceId;
    private final DoubleArraySubscriber m_observationSubscriber;
    private final DoubleArraySubscriber m_objDetectObservationSubscriber;
    private final IntegerSubscriber m_fpsAprilTagsSubscriber;
    private final IntegerSubscriber m_fpsObjDetectSubscriber;
    private final StringPublisher m_eventNamePublisher;
    private final IntegerPublisher m_matchTypePublisher;
    private final IntegerPublisher m_matchNumberPublisher;
    private final IntegerPublisher m_timestampPublisher;
    private final BooleanPublisher m_isRecordingPublisher;
    private final StringPublisher m_tagLayoutPublisher;

    private final Timer m_slowPeriodicTimer = new Timer();

    public VisionIOMackinac(AprilTagFieldLayout fieldLayout, int index) {
        m_fieldLayout = fieldLayout;
        m_deviceId = "mackinac_" + index;
        var mackinacTable = NetworkTableInstance.getDefault().getTable(m_deviceId);
        var configTable = mackinacTable.getSubTable("config");
        var camera = cameras[index];

        configTable.getStringTopic("camera_id").publish().set(camera.id());
        configTable.getIntegerTopic("camera_resolution_width").publish().set(camera.width());
        configTable.getIntegerTopic("camera_resolution_height").publish().set(camera.height());
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(camera.autoExposure());
        configTable.getIntegerTopic("camera_exposure").publish().set(camera.exposure());
        configTable.getDoubleTopic("camera_gain").publish().set(camera.gain());
        configTable.getDoubleTopic("camera_denoise").publish().set(camera.denoise());
        configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.APRIL_TAG_WIDTH.in(Meters));
        m_isRecordingPublisher = configTable.getBooleanTopic("is_recording").publish();
        m_isRecordingPublisher.set(false);
        m_timestampPublisher = configTable.getIntegerTopic("timestamp").publish();
        m_tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();
        m_eventNamePublisher = configTable.getStringTopic("event_name").publish();
        m_matchTypePublisher = configTable.getIntegerTopic("match_type").publish();
        m_matchNumberPublisher = configTable.getIntegerTopic("match_number").publish();

        var outputTable = mackinacTable.getSubTable("output");
        m_observationSubscriber = outputTable
                .getDoubleArrayTopic("observations")
                .subscribe(
                        new double[] {},
                        PubSubOption.keepDuplicates(true),
                        PubSubOption.sendAll(true),
                        PubSubOption.pollStorage(5),
                        PubSubOption.periodic(0.01));
        m_objDetectObservationSubscriber = outputTable
                .getDoubleArrayTopic("objdetect_observations")
                .subscribe(
                        new double[] {},
                        PubSubOption.keepDuplicates(true),
                        PubSubOption.sendAll(true),
                        PubSubOption.pollStorage(5),
                        PubSubOption.periodic(0.01));
        m_fpsAprilTagsSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);
        m_fpsObjDetectSubscriber = outputTable.getIntegerTopic("fps_objdetect").subscribe(0);

        var cameraTable = cameraPublisherTable.getSubTable("Camera" + index);
        cameraTable.getBooleanTopic("connected").publish().set(true);
        cameraTable.getStringTopic("description").publish().set("Mackinac Camera" + index);
        {
            final String mode = "" + camera.width() + "x" + camera.height() + " MJPEG 60 fps";
            cameraTable.getStringTopic("mode").publish().set(mode);
            cameraTable.getStringArrayTopic("modes").publish().set(new String[] {mode});
        }
        {
            final String source = "mjpg:http://10.71.66.14:580" + (index * 2 + 1) + "/stream.mjpg";
            cameraTable.getStringTopic("source").publish().set(source);
            cameraTable.getStringArrayTopic("streams").publish().set(new String[] {source});
        }

        m_slowPeriodicTimer.start();

        String layoutString;
        try {
            layoutString = new ObjectMapper().writeValueAsString(m_fieldLayout);
        } catch (JsonProcessingException e) {
            throw new RuntimeException("Failed to serialize AprilTag layout JSON " + toString() + "for mackinac");
        }

        m_tagLayoutPublisher.set(layoutString);
    }

    public void updateInputs(
            VisionIOInputs inputs, AprilTagVisionIOInputs aprilTagInputs, ObjDetectVisionIOInputs objDetectInputs) {
        boolean slowPeriodic = m_slowPeriodicTimer.advanceIfElapsed(1.0);

        // Update NT connection status
        inputs.ntConnected = false;
        for (var client : NetworkTableInstance.getDefault().getConnections()) {
            if (client.remote_id.startsWith(m_deviceId)) {
                inputs.ntConnected = true;
                break;
            }
        }

        // Publish timestamp
        // if (slowPeriodic && SystemTimeValidReader.isValid())
        if (slowPeriodic) m_timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1000000);

        if (slowPeriodic) {
            m_eventNamePublisher.set(DriverStation.getEventName());
            m_matchTypePublisher.set(DriverStation.getMatchType().ordinal());
            m_matchNumberPublisher.set(DriverStation.getMatchNumber());
        }

        // Publish tag layout
        // var aprilTagType = aprilTagLayoutSupplier.get();
        // if (aprilTagType != lastAprilTagLayout) {
        //     lastAprilTagLayout = aprilTagType;
        //     tagLayoutPublisher.set(aprilTagType.getLayoutString());
        // }
        // ^ done in constructor

        // Get AprilTag data
        var aprilTagQueue = m_observationSubscriber.readQueue();
        aprilTagInputs.timestamps = new double[aprilTagQueue.length];
        aprilTagInputs.frames = new double[aprilTagQueue.length][];
        for (int i = 0; i < aprilTagQueue.length; i++) {
            aprilTagInputs.timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
            aprilTagInputs.frames[i] = aprilTagQueue[i].value;
        }
        if (slowPeriodic) {
            aprilTagInputs.fps = m_fpsAprilTagsSubscriber.get();
        }

        // Get object detection data
        var objDetectQueue = m_objDetectObservationSubscriber.readQueue();
        objDetectInputs.timestamps = new double[objDetectQueue.length];
        objDetectInputs.frames = new double[objDetectQueue.length][];
        for (int i = 0; i < objDetectQueue.length; i++) {
            objDetectInputs.timestamps[i] = objDetectQueue[i].timestamp / 1000000.0;
            objDetectInputs.frames[i] = objDetectQueue[i].value;
        }
        if (slowPeriodic) {
            objDetectInputs.fps = m_fpsObjDetectSubscriber.get();
        }
    }

    @Override
    public void setRecording(boolean shouldRecord) {
        m_isRecordingPublisher.set(shouldRecord);
    }
}
