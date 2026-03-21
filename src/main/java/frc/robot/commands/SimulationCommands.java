package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public final class SimulationCommands {
    public static class SimFuelCommand extends Command {
        private static ArrayList<Pair<Supplier<Pose3d>, Timer>> poseList = new ArrayList<>();
        public static void step() {
            Logger.recordOutput("SimFuel", poseList
                .stream()
                .map(v -> v.getFirst().get())
                .collect(Collectors.toList()).toArray(Pose3d[]::new));

            poseList.removeIf(pair -> pair.getSecond().hasElapsed(5d));
        }

        private final Timer m_timer = new Timer();

        private Pose3d m_initialPose = new Pose3d();
        private Pose3d m_pose = m_initialPose;
        private Translation3d m_initialVelocity;

        private Supplier<Pose3d> m_initialPoseSupplier;
        private Supplier<Translation3d> m_initialVelocitySupplier;

        public SimFuelCommand(Supplier<Pose3d> startPoseSupplier, Supplier<Translation3d> initialVelocitySupplier) {
            m_initialPoseSupplier = startPoseSupplier;
            m_initialVelocitySupplier = initialVelocitySupplier;

            poseList.add(new Pair<>(() -> m_pose, m_timer));
        }

        @Override
        public void initialize() {
            m_timer.restart();

            m_initialPose = m_initialPoseSupplier.get();
            m_pose = m_initialPose;
            m_initialVelocity = m_initialVelocitySupplier.get();
        }

        @Override
        public void execute() {
            final double t = m_timer.get();

            final double xpos = m_initialPose.getX() + m_initialVelocity.getX() * t;
            final double ypos = m_initialPose.getY() + m_initialVelocity.getY() * t;
            final double zpos = m_initialPose.getZ()
                    + m_initialVelocity.getZ() * t
                    - 4.905d * t * t;

            m_pose = new Pose3d(new Translation3d(xpos, ypos, zpos), Rotation3d.kZero);
        }

        @Override
        public boolean isFinished() {
            return m_pose.getZ() < 0d;
        }
    }
}
