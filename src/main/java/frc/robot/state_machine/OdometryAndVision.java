package frc.robot.state_machine;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.ApriltagUtil;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class OdometryAndVision {
    private Rotation2d gyroOffset = Rotation2d.kZero;

    public record OdometryObservation(
            SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

    public record TxTyObservation(int tagId, int camera, double[] tx, double[] ty, double distance, double timestamp) {}

    public record TxTyPoseRecord(Pose2d pose, double distance, double timestamp) {}

    public record FuelTxTyObservation(int camera, double[] tx, double[] ty, double timestamp) {}

    public record FuelPoseRecord(Translation2d translation, double timestamp) {}

    private static final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    private static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

    static {
        for (int i = 0; i < 3; ++i) qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }

    private static final double poseBufferSizeSeconds = 2d;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private Pose2d odometryPose = Pose2d.kZero;
    private Pose2d estimatedPose = Pose2d.kZero;

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public Rotation2d getRotation() {
        return estimatedPose.getRotation();
    }

    public void resetPose(Pose2d pose) {
        gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        estimatedPose = pose;
        odometryPose = pose;
        poseBuffer.clear();
    }

    public void addOdometryObservation(OdometryObservation observation) {
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist);
        // Use gyro if connected
        observation.gyroAngle.ifPresent(gyroAngle -> {
            Rotation2d angle = gyroAngle.plus(gyroOffset);
            odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // Calculate diff from last odometry pose and add onto pose estimate
        Twist2d finalTwist = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(finalTwist);
    }

    public void addVisionObservation(VisionObservation observation) {
        // if we are somehow outside of the field (I think this happens because we are trusting odometry too much and we
        // have had wheel slip on the bump),
        // accept the raw vision pose without question
        if (estimatedPose.getX()
                        > FieldConstants.FIELD_LENGTH
                                .plus(DriveConstants.BUMPER_LARGEST_DIMENSION)
                                .in(Meters)
                || estimatedPose.getY()
                        > FieldConstants.FIELD_WIDTH
                                .plus(DriveConstants.BUMPER_LARGEST_DIMENSION)
                                .in(Meters)) {
            estimatedPose = observation.visionPose;
            return;
        }

        // If the measurement is old enough to be outside the pose buffer's timespan, skip
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds > observation.timestamp()) return;
        } catch (NoSuchElementException ex) {
            return;
        }

        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) return;

        // sample --> odometryPose transform and backwards of that
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
        Pose2d finalPose = estimateAtTime;

        // if (Constants.USE_MACKINAC || StateMachine.isShooting()) {
        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i)
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }
        // difference between estimate and vision pose
        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
        // scale transform by visionK
        var kTimesTransform = visionK.times(VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform = new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        finalPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
        // } else {
        // Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
        // finalPose = estimateAtTime.plus(transform).plus(sampleToOdometryTransform);
        // }

        // reject poses outside of field

        if (finalPose.getX() < 0d || finalPose.getY() < 0d) return;

        if (finalPose.getX() > FieldConstants.FIELD_LENGTH.in(Meters)
                || finalPose.getY() > FieldConstants.FIELD_WIDTH.in(Meters)) return;

        estimatedPose = finalPose;
    }

    private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();
    private Set<FuelPoseRecord> fuelPoses = new HashSet<>();
    private final double fuelPersistanceTime = 0.75d;
    private final Map<Integer, Pose2d> tagPoses2d = ApriltagUtil.aprilTagMap.entrySet().stream()
            .map(e -> Map.entry(e.getKey(), e.getValue().pose.toPose2d()))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));

    public void addTxTyObservation(TxTyObservation observation) {
        // Skip if current data for tag is newer
        if (txTyPoses.containsKey(observation.tagId())
                && txTyPoses.get(observation.tagId()).timestamp() >= observation.timestamp()) return;

        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) return;

        Rotation2d robotRotation = estimatedPose
                .transformBy(new Transform2d(odometryPose, sample.get()))
                .getRotation();

        // Average tx's and ty's
        double tx = 0.0;
        double ty = 0.0;
        for (int i = 0; i < 4; i++) {
            tx += observation.tx()[i];
            ty += observation.ty()[i];
        }
        tx /= 4.0;
        ty /= 4.0;

        Pose3d cameraPose = VisionConstants.cameras[observation.camera()].pose().get();

        // Use 3D distance and tag angles to find robot pose
        Translation2d camToTagTranslation = new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, -tx))
                .transformBy(new Transform3d(new Translation3d(observation.distance(), 0, 0), Rotation3d.kZero))
                .getTranslation()
                .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
                .toTranslation2d();
        Rotation2d camToTagRotation =
                robotRotation.plus(cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));
        var tagPose2d = tagPoses2d.get(observation.tagId());
        if (tagPose2d == null) return;

        Translation2d fieldToCameraTranslation = new Pose2d(
                        tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0d, Rotation2d.kZero))
                .getTranslation();
        Pose2d robotPose = new Pose2d(
                        fieldToCameraTranslation,
                        robotRotation.plus(cameraPose.toPose2d().getRotation()))
                .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
        // Use gyro angle at time for robot rotation
        robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

        // Add transform to current odometry based pose for latency correction
        txTyPoses.put(
                observation.tagId(),
                new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(), observation.timestamp()));
    }

    public void addFuelTxTyObservation(FuelTxTyObservation observation) {
        var oldOdometryPose = poseBuffer.getSample(observation.timestamp());
        if (oldOdometryPose.isEmpty()) return;

        Pose2d fieldToRobot = estimatedPose.transformBy(new Transform2d(odometryPose, oldOdometryPose.get()));
        Pose3d robotToCamera =
                VisionConstants.cameras[observation.camera()].pose().get();

        // // Assume fuel height of zero and find midpoint of width of bottom tx ty
        double tx = (observation.tx()[2] + observation.tx()[3]) / 2;
        double ty = (observation.ty()[2] + observation.ty()[3]) / 2;

        double cameraToFuelAngle = -robotToCamera.getRotation().getY() - ty;
        // System.out.println(cameraToFuelAngle);
        if (cameraToFuelAngle >= 0) return;

        double cameraToFuelNorm = (-robotToCamera.getZ()) / Math.tan(cameraToFuelAngle) / Math.cos(-tx);
        Pose2d robotToCamera2d = robotToCamera.toPose2d();
        Pose2d fieldToCamera = fieldToRobot.transformBy(
                new Transform2d(robotToCamera2d.getTranslation(), robotToCamera2d.getRotation()));
        Pose2d fieldToFuel = fieldToCamera
                .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-tx)))
                .transformBy(new Transform2d(new Translation2d(cameraToFuelNorm, 0), Rotation2d.kZero));
        Translation2d fieldToFuelTranslation2d = fieldToFuel.getTranslation();

        FuelPoseRecord fuelPoseRecord = new FuelPoseRecord(fieldToFuelTranslation2d, observation.timestamp());

        fuelPoses = fuelPoses.stream()
                .filter((x) -> x.translation.getDistance(fieldToFuelTranslation2d) > 0.5d)
                .collect(Collectors.toSet());
        fuelPoses.add(fuelPoseRecord);
    }

    public Set<Translation2d> getFuelTranslations() {
        return fuelPoses.stream()
                .filter((x) -> Timer.getTimestamp() - x.timestamp() < fuelPersistanceTime)
                .map(FuelPoseRecord::translation)
                .collect(Collectors.toSet());
    }
}
