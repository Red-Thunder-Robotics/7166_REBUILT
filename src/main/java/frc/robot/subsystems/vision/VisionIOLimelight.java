package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import frc.robot.state_machine.OdometryAndVision.VisionObservation;
import frc.robot.state_machine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public final class VisionIOLimelight implements VisionIO {
    public VisionIOLimelight() {
        for (int port = 5800; port <= 5809; port++)
            PortForwarder.add(port, limelightFrontLeftName + ".local", port);

        for (int port = 5800; port <= 5809; port++)
            PortForwarder.add(port + 10, limelightBackRightName + ".local", port);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        boolean megaTagSuccess = updateVisionMegaTag2(limelightFrontLeftName);
        {
            var success = updateVisionMegaTag2(limelightBackRightName);
            megaTagSuccess |= success;
        }

        inputs.megaTagSuccess = megaTagSuccess;
    }

    
    // private final Matrix<N3, N1> stdDevs = VecBuilder.fill(.7,.7,9999999);
    private final Matrix<N3, N1> stdDevs = VecBuilder.fill(.35,.35,9999999);
    private boolean updateVisionMegaTag2(String limelightName) {
        final Drive drive = Drive.instance;

        if (Math.abs(drive.getYawVelocityRadPerSec()) > (4 * Math.PI))
            return false;

        // final double yaw_degrees = StateMachine.odometryAndVision.getRotation().getDegrees();
        final double yaw_degrees = StateMachine.odometryAndVision.getOdometryPose().getRotation().getDegrees();

        LimelightHelpers.SetRobotOrientation(limelightName, yaw_degrees, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if(mt2.tagCount == 0)
            return false;

        Logger.recordOutput("Vision/" + limelightName + "/Pose", mt2.pose);
        StateMachine.odometryAndVision.addVisionObservation(new VisionObservation(mt2.pose, mt2.timestampSeconds, stdDevs));

        return true;
    }

}
