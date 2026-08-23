package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        boolean megaTagSuccess;
        boolean ntConnected;
    }

    @AutoLog
    class AprilTagVisionIOInputs {
        double[] timestamps = new double[] {};
        double[][] frames = new double[][] {};
        long fps = 0;
    }

    @AutoLog
    class ObjDetectVisionIOInputs {
        double[] timestamps = new double[] {};
        double[][] frames = new double[][] {};
        long fps = 0;
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default void updateInputs(
            VisionIOInputs inputs, AprilTagVisionIOInputs aprilTagInputs, ObjDetectVisionIOInputs objDetectInputs) {}

    default void setRecording(boolean shouldRecord) {}
}
