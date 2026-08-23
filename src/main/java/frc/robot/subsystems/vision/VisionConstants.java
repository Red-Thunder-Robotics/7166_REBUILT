package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import java.util.function.Supplier;

public final class VisionConstants {
    public static final String limelightFrontName = "limelight-front";
    public static final String limelightLeftName = "limelight-left";
    public static final String limelightRightName = "limelight-right";
    public static final String[] limelightList = {limelightFrontName, limelightLeftName, limelightRightName};
    // public static final String[] limelightPoseEstimationList = {
    //     limelightFrontName,
    //     limelightLeftName
    // };
    public static final String[] limelightPoseEstimationList = limelightList;

    private static final boolean forceEnableInstanceLogging = true;

    @SuppressWarnings("unused")
    public static final boolean enableInstanceLogging =
            forceEnableInstanceLogging || Constants.CURRENT_MODE == Constants.Mode.REPLAY;

    public static final double ambiguityThreshold = 0.4d;
    public static final double targetLogTimeSecs = 0.1d;
    public static final double fieldBorderMargin = 0.5d;
    public static final double xyStdDevCoefficient = 0.01d;
    public static final double thetaStdDevCoefficient = 0.03d;
    public static final double demoTagPosePersistenceSecs = 0.5d;
    public static final double fuelDetectConfidenceThreshold = 0.35d;
    public static final double timestampOffset = 0d;

    // cameras 1 and 3
    private static final Distance frontCameraX = Inches.of(0.997d);
    private static final Distance frontCameraY = Inches.of(10.065d);
    private static final Distance frontCameraZ = Inches.of(20.477d);
    private static final Angle frontCameraRoll = Degrees.of(0d);
    private static final Angle frontCameraPitch = Degrees.of(-8d);
    private static final Angle frontCameraYaw = Degrees.of(20d);

    public record CameraConfig(
            Supplier<Pose3d> pose,
            String id,
            int width,
            int height,
            int autoExposure,
            int exposure,
            double gain,
            double denoise,
            double stdDevFactor) {}

    public static CameraConfig[] cameras = new CameraConfig[] {
        // new CameraConfig( // logic 2
        //     () -> new Pose3d(
        //         -0.2794,
        //         0.2286,
        //         0.21113,
        //         new Rotation3d(
        //             Degrees.of(0d),
        //             // Units.degreesToRadians(-25d + pitchAdjustments[0].get()),
        //             Degrees.of(30d),
        //             // Units.degreesToRadians(-20d))),
        //             Degrees.of(180d))),
        //     // "0x1120:0x46D:0x825",
        //     "0x3120:0x46D:0x825",
        //     1280,
        //     720,
        //     0,
        //     200,
        //     30,
        //     1d,
        //     1d
        // )
        new CameraConfig( // thriftycam0
                () -> new Pose3d(
                        frontCameraX,
                        frontCameraY.unaryMinus(),
                        frontCameraZ,
                        new Rotation3d(frontCameraRoll, frontCameraPitch, frontCameraYaw)),
                "0x3220:0x5C8:0xA00",
                1600,
                1304,
                0,
                50,
                10,
                1d,
                1d),
        new CameraConfig( // thriftycam1
                () -> new Pose3d(
                        frontCameraX,
                        frontCameraY.unaryMinus(),
                        frontCameraZ,
                        new Rotation3d(frontCameraRoll, frontCameraPitch, frontCameraYaw)),
                "0x3100:0x5C8:0xA00",
                1600,
                1304,
                0,
                50,
                10,
                1d,
                1d),
        new CameraConfig( // thriftycam2
                () -> new Pose3d(
                        frontCameraX,
                        frontCameraY.unaryMinus(),
                        frontCameraZ,
                        new Rotation3d(frontCameraRoll, frontCameraPitch, frontCameraYaw)),
                "0x3220:0x5C8:0xA00",
                1600,
                1304,
                0,
                50,
                10,
                1d,
                1d),
        new CameraConfig( // thriftycam3
                () -> new Pose3d(
                        frontCameraX,
                        frontCameraY,
                        frontCameraZ,
                        new Rotation3d(frontCameraRoll, frontCameraPitch, frontCameraYaw.unaryMinus())),
                "0x1000:0x5C8:0xA00",
                1600,
                1304,
                0,
                50,
                10,
                1d,
                1d),
    };
}
