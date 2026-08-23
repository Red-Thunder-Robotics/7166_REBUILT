package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;

public final class ApriltagUtil {
    public static final HashMap<Integer, AprilTag> aprilTagMap = new HashMap<>();
    public static AprilTagFieldLayout fieldLayout = null;
    public static boolean aprilTagFieldLayoutSuccess = false;
    public static double fieldLength = 0d;
    public static double fieldWidth = 0d;

    static {
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            for (var tag : fieldLayout.getTags()) aprilTagMap.put(tag.ID, tag);
            aprilTagFieldLayoutSuccess = true;

            fieldLength = fieldLayout.getFieldLength();
            fieldWidth = fieldLayout.getFieldWidth();
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout: " + e.getMessage(), false);
        }
    }
}
