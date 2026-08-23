package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machine.OdometryAndVision.FuelTxTyObservation;
import frc.robot.state_machine.OdometryAndVision.TxTyObservation;
import frc.robot.state_machine.OdometryAndVision.VisionObservation;
import frc.robot.state_machine.StateMachine;
import frc.robot.util.ApriltagUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public final class VisionSubsystem extends SubsystemBase {
    public static VisionSubsystem instance = null;

    private final AprilTagFieldLayout m_fieldLayout;

    private final boolean m_useSingle;
    private final VisionIO m_single_io;
    private final VisionIO[] m_multi_io;
    private final VisionIOInputsAutoLogged[] m_inputs;
    private final AprilTagVisionIOInputsAutoLogged[] m_aprilTagInputs;
    private final ObjDetectVisionIOInputsAutoLogged[] m_objDetectInputs;

    private final LoggedNetworkBoolean m_recordingRequest =
            new LoggedNetworkBoolean("/SmartDashboard/Enable Recording", false);

    private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    private Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>();
    private List<FuelTxTyObservation> allFuelTxTyObservations = new ArrayList<>();

    // single (Limelight, PhotonVision, etc)
    public VisionSubsystem(VisionIO io) {
        instance = this;
        m_fieldLayout = null;

        m_useSingle = true;

        m_multi_io = null;
        m_single_io = io;
        m_inputs = new VisionIOInputsAutoLogged[] {new VisionIOInputsAutoLogged()};
        m_aprilTagInputs = null;
        m_objDetectInputs = null;
    }
    // multi (mackinac)
    public VisionSubsystem(AprilTagFieldLayout fieldLayout, VisionIO... io) {
        instance = this;
        m_fieldLayout = fieldLayout;

        m_useSingle = false;

        m_single_io = null;
        m_multi_io = io;
        m_inputs = new VisionIOInputsAutoLogged[io.length];
        m_aprilTagInputs = new AprilTagVisionIOInputsAutoLogged[io.length];
        m_objDetectInputs = new ObjDetectVisionIOInputsAutoLogged[io.length];

        for (int i = 0; i < io.length; i++) {
            m_inputs[i] = new VisionIOInputsAutoLogged();
            m_aprilTagInputs[i] = new AprilTagVisionIOInputsAutoLogged();
            m_objDetectInputs[i] = new ObjDetectVisionIOInputsAutoLogged();

            lastFrameTimes.put(i, 0d);
        }
    }

    @Override
    public void periodic() {
        if (m_useSingle) {
            // NOTE: m_fieldLayout is null
            var input = m_inputs[0];
            m_single_io.updateInputs(input);
            Logger.processInputs("Vision", input);
        } else {
            for (int i = 0; i < m_multi_io.length; i++) {
                m_multi_io[i].updateInputs(m_inputs[i], m_aprilTagInputs[i], m_objDetectInputs[i]);
                Logger.processInputs("Vision/Inst" + i, m_inputs[i]);
                Logger.processInputs("AprilTagVision/Inst" + i, m_aprilTagInputs[i]);
                Logger.processInputs("ObjDetectVision/Inst" + i, m_objDetectInputs[i]);
            }

            if (!m_recordingRequest.get() && DriverStation.isFMSAttached()) m_recordingRequest.set(true);
            final boolean shouldRecord = m_recordingRequest.get();
            for (var io : m_multi_io) io.setRecording(shouldRecord);

            allTxTyObservations.clear();
            allFuelTxTyObservations.clear();

            // Loop over instances
            List<Pose2d> allRobotPoses = new ArrayList<>();
            List<VisionObservation> allVisionObservations = new ArrayList<>();
            for (int instanceIndex = 0; instanceIndex < m_multi_io.length; instanceIndex++) {
                // Loop over frames
                for (int frameIndex = 0; frameIndex < m_aprilTagInputs[instanceIndex].timestamps.length; frameIndex++) {
                    lastFrameTimes.put(instanceIndex, Timer.getTimestamp());
                    var timestamp = m_aprilTagInputs[instanceIndex].timestamps[frameIndex] + timestampOffset;
                    var values = m_aprilTagInputs[instanceIndex].frames[frameIndex];

                    // Exit if blank frame
                    if (values.length == 0 || values[0] == 0) continue;

                    // Switch based on number of poses
                    Pose3d cameraPose = null;
                    Pose2d robotPose = null;
                    boolean useVisionRotation = false;
                    switch ((int) values[0]) {
                        case 1: {
                            // One pose (multi-tag), use directly
                            cameraPose = new Pose3d(
                                    values[2],
                                    values[3],
                                    values[4],
                                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                            final Pose2d pose2d =
                                    cameras[instanceIndex].pose().get().toPose2d();
                            robotPose = cameraPose
                                    .toPose2d()
                                    .transformBy(
                                            new Transform2d(pose2d.getTranslation(), pose2d.getRotation()).inverse());
                            useVisionRotation = true;
                            break;
                        }
                        case 2: {
                            // Two poses (one tag), disambiguate
                            double error0 = values[1];
                            double error1 = values[9];
                            Pose3d cameraPose0 = new Pose3d(
                                    values[2],
                                    values[3],
                                    values[4],
                                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                            Pose3d cameraPose1 = new Pose3d(
                                    values[10],
                                    values[11],
                                    values[12],
                                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
                            final Pose2d pose2d =
                                    cameras[instanceIndex].pose().get().toPose2d();
                            Transform2d cameraToRobot =
                                    new Transform2d(pose2d.getTranslation(), pose2d.getRotation()).inverse();
                            Pose2d robotPose0 = cameraPose0.toPose2d().transformBy(cameraToRobot);
                            Pose2d robotPose1 = cameraPose1.toPose2d().transformBy(cameraToRobot);

                            // Check for ambiguity and select based on estimated rotation
                            if (error0 < error1 * ambiguityThreshold || error1 < error0 * ambiguityThreshold) {
                                Rotation2d currentRotation = StateMachine.odometryAndVision.getRotation();
                                Rotation2d visionRotation0 = robotPose0.getRotation();
                                Rotation2d visionRotation1 = robotPose1.getRotation();
                                if (Math.abs(currentRotation
                                                .minus(visionRotation0)
                                                .getRadians())
                                        < Math.abs(currentRotation
                                                .minus(visionRotation1)
                                                .getRadians())) {
                                    cameraPose = cameraPose0;
                                    robotPose = robotPose0;
                                } else {
                                    cameraPose = cameraPose1;
                                    robotPose = robotPose1;
                                }
                            }
                            break;
                        }
                    }

                    // Exit if no data
                    if (cameraPose == null || robotPose == null) continue;

                    // Exit if robot pose is off the field
                    if (robotPose.getX() < -fieldBorderMargin
                            || robotPose.getX() > ApriltagUtil.fieldLength + fieldBorderMargin
                            || robotPose.getY() < -fieldBorderMargin
                            || robotPose.getY() > ApriltagUtil.fieldWidth + fieldBorderMargin) continue;

                    // Get tag poses and update last detection times
                    List<Pose3d> tagPoses = new ArrayList<>();
                    for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i += 10) {
                        int tagId = (int) values[i];
                        lastTagDetectionTimes.put(tagId, Timer.getTimestamp());
                        Optional<Pose3d> tagPose = m_fieldLayout.getTagPose((int) values[i]);
                        tagPose.ifPresent(tagPoses::add);
                    }
                    if (tagPoses.isEmpty()) continue;

                    // Calculate average distance to tag
                    double totalDistance = 0.0;
                    for (Pose3d tagPose : tagPoses)
                        totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                    double avgDistance = totalDistance / tagPoses.size();

                    // Add observation to list
                    double xyStdDev = xyStdDevCoefficient
                            * Math.pow(avgDistance, 1.2)
                            / Math.pow(tagPoses.size(), 2.0)
                            * cameras[instanceIndex].stdDevFactor();
                    double thetaStdDev = useVisionRotation
                            ? thetaStdDevCoefficient
                                    * Math.pow(avgDistance, 1.2)
                                    / Math.pow(tagPoses.size(), 2.0)
                                    * cameras[instanceIndex].stdDevFactor()
                            : Double.POSITIVE_INFINITY;
                    allVisionObservations.add(new VisionObservation(
                            robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
                    allRobotPoses.add(robotPose);

                    // Log data from instance
                    if (enableInstanceLogging) {
                        Logger.recordOutput(
                                "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
                                Timer.getTimestamp() - timestamp);
                        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);
                        Logger.recordOutput(
                                "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
                    }
                }

                // Get tag tx ty observation data
                Map<Integer, TxTyObservation> txTyObservations = new HashMap<>();
                for (int frameIndex = 0; frameIndex < m_aprilTagInputs[instanceIndex].timestamps.length; frameIndex++) {
                    var timestamp = m_aprilTagInputs[instanceIndex].timestamps[frameIndex];
                    var values = m_aprilTagInputs[instanceIndex].frames[frameIndex];
                    int tagEstimationDataEndIndex =
                            switch ((int) values[0]) {
                                default -> 0;
                                case 1 -> 8;
                                case 2 -> 16;
                            };

                    for (int index = tagEstimationDataEndIndex + 1; index < values.length; index += 10) {
                        double[] tx = new double[4];
                        double[] ty = new double[4];
                        for (int i = 0; i < 4; i++) {
                            tx[i] = values[index + 1 + (2 * i)];
                            ty[i] = values[index + 1 + (2 * i) + 1];
                        }
                        int tagId = (int) values[index];
                        double distance = values[index + 9];

                        txTyObservations.put(
                                tagId, new TxTyObservation(tagId, instanceIndex, tx, ty, distance, timestamp));
                    }
                }

                // Save tx ty observation data
                for (var observation : txTyObservations.values()) {
                    if (!allTxTyObservations.containsKey(observation.tagId())
                            || observation.distance()
                                    < allTxTyObservations
                                            .get(observation.tagId())
                                            .distance()) {
                        allTxTyObservations.put(observation.tagId(), observation);
                    }
                }

                // Record object detection observations
                /*
                switch (instanceIndex) {
                    case 2:
                        // Record coral observations
                        for (int frameIndex = 0;
                            frameIndex < m_objDetectInputs[instanceIndex].timestamps.length;
                            frameIndex++) {
                            double[] frame = m_objDetectInputs[instanceIndex].frames[frameIndex];
                            for (int i = 0; i < frame.length; i += 10) {
                                if (frame[i + 1] > coralDetectConfidenceThreshold) {
                                    double[] tx = new double[4];
                                    double[] ty = new double[4];
                                    for (int z = 0; z < 4; z++) {
                                        tx[z] = frame[i + 2 + (2 * z)];
                                        ty[z] = frame[i + 2 + (2 * z) + 1];
                                    }
                                    allCoralTxTyObservations.add(
                                        new CoralTxTyObservation(
                                            instanceIndex,
                                            tx,
                                            ty,
                                            objDetectInputs[instanceIndex].timestamps[frameIndex]));
                                }
                            }
                        }
                    break;

                    case 0:
                    case 1:
                    // Record algae observations
                    if (m_objDetectInputs[instanceIndex].timestamps.length > 0) {
                        double[] frame =
                            m_objDetectInputs[instanceIndex]
                                .frames[m_objDetectInputs[instanceIndex].timestamps.length - 1];
                        double largestSize = 0.0;
                        Rotation2d angle = null;
                        for (int i = 0; i < frame.length; i += 10) {
                            if (frame[i + 1] > algaeDetectConfidenceThreshold) {
                                // Read data from frame
                                double[] tx = new double[4];
                                double[] ty = new double[4];
                                for (int z = 0; z < 4; z++) {
                                    tx[z] = frame[i + 2 + (2 * z)];
                                    ty[z] = frame[i + 2 + (2 * z) + 1];
                                }

                                // Check if at bottom of frame
                                double maxTy = Math.max(ty[2], ty[3]);
                                if (maxTy < 0.4)
                                    continue;

                                // Check if largest algae
                                double size = Math.abs(tx[0] - tx[1]);
                                if (size > largestSize) {
                                    largestSize = size;
                                    angle = Rotation2d.fromRadians(-(tx[0] + tx[1]) / 2.0);
                                }
                            }
                        }
                        if (angle != null) {
                            if (instanceIndex == 0)
                                OurRobotState.addLeftAlgaeObservation(angle);
                            else
                                OurRobotState.addRightAlgaeObservation(angle);
                        }
                    }
                    break;
                }
                */
                switch (instanceIndex) {
                    case 1:
                        for (int frameIndex = 0;
                                frameIndex < m_objDetectInputs[instanceIndex].timestamps.length;
                                frameIndex++) {
                            double[] frame = m_objDetectInputs[instanceIndex].frames[frameIndex];
                            for (int i = 0; i < frame.length; i += 10) {
                                if (frame[i + 1] > fuelDetectConfidenceThreshold) {
                                    double[] tx = new double[4];
                                    double[] ty = new double[4];
                                    for (int z = 0; z < 4; z++) {
                                        tx[z] = frame[i + 2 + (2 * z)];
                                        ty[z] = frame[i + 2 + (2 * z) + 1];
                                    }
                                    allFuelTxTyObservations.add(new FuelTxTyObservation(
                                            instanceIndex,
                                            tx,
                                            ty,
                                            m_objDetectInputs[instanceIndex].timestamps[frameIndex]));
                                }
                            }
                        }
                        break;

                    default:
                        break;
                }

                // If no frames from instances, clear robot pose
                if (enableInstanceLogging && m_aprilTagInputs[instanceIndex].timestamps.length == 0)
                    Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", Pose2d.kZero);

                // If no recent frames from instance, clear tag poses
                if (enableInstanceLogging
                        && Timer.getTimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
                    Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/TagPoses", new Pose3d[] {});
                }
            }

            // Log robot poses
            // Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));

            // Log tag poses
            List<Pose3d> allTagPoses = new ArrayList<>();
            for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
                if (Timer.getTimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
                    m_fieldLayout.getTagPose(detectionEntry.getKey()).ifPresent(allTagPoses::add);
                }
            }
            Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

            // Send results to robot state
            allVisionObservations.stream()
                    .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
                    .forEach(StateMachine.odometryAndVision::addVisionObservation);
            allTxTyObservations.values().stream().forEach(StateMachine.odometryAndVision::addTxTyObservation);
            allFuelTxTyObservations.stream()
                    .sorted(Comparator.comparingDouble(FuelTxTyObservation::timestamp))
                    .forEach(StateMachine.odometryAndVision::addFuelTxTyObservation);

            Logger.recordOutput(
                    "Vision/CameraPoses",
                    Arrays.stream(cameras).map(x -> x.pose().get()).toArray(Pose3d[]::new));

            // Record cycle time
            // LoggedTracer.record("Vision");
        }
    }
}
