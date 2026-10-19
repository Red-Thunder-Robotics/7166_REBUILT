package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.state_machine.StateMachine;
import frc.robot.subsystems.drive.Drive;

/**
 * Toy model of the robot driving into a clump of fuel, for simulation only.
 *
 * None of these numbers were measured on the robot. They were picked so the
 * existing R8 auto bogs down in the clump at its 2.3 m/s sweep speed, which is
 * what the team reports on the real robot. Use it to check that beach detection
 * and recovery logic fires and behaves sensibly, not to predict real speeds.
 *
 * Model: fuel that reaches the intake mouth joins a "jam" count. The rollers
 * pull jammed fuel in at a rate that falls as the jam grows (torque limit).
 * When fuel arrives faster than the rollers take it, the jam grows, the rollers
 * slow, and past a threshold the robot rides up on the pile and loses traction.
 * Backing away from the pile releases jammed fuel back onto the field.
 */
public final class SimFuelClump {
    // Clump placed where the R8 auto sweeps (blue alliance, field meters)
    private static final double clumpMinX = 7.25, clumpMaxX = 8.45;
    private static final double clumpMinY = 2.0, clumpMaxY = 4.4;
    private static final double fuelSpacing = 0.16; // fuel is 0.15 m across

    // Robot geometry. Intake is on the back (-x) side, which is the side the
    // paths drive into fuel with.
    private static final double intakeSide = -1d;
    private static final double bumperHalf = 0.44;
    private static final double mouthHalfWidth = 0.35;
    private static final double mouthDepth = 0.25; // how far the deployed intake reaches past the bumper
    private static final double deployedInches = 10d; // actuator past this counts as deployed

    // Invented intake behavior
    private static final double ingestCapacity = 40d; // fuel per second at full roller speed
    private static final double jamStall = 14d; // jam size that stalls the rollers
    private static final double beachStart = 6d; // jam size where the robot starts riding up
    private static final double beachFull = 12d; // jam size where it has lost nearly all traction
    private static final double pitchPerFuel = 1.5d; // degrees of tilt per jammed fuel past beachStart
    private static final double releasePerMeter = 25d; // jammed fuel released per meter of backing away

    private static final List<Translation2d> fuel = new ArrayList<>();
    private static double jam = 0d;
    private static double ingested = 0d;
    private static double releaseCarry = 0d;

    private static double actuatorInches = 0d;
    private static double rollerTargetRPS = 0d;
    private static double rollerFreeRPS = 1d;

    private static final boolean enabled = !"0".equals(System.getenv("SIM_FUEL_CLUMP"));

    static {
        if (enabled)
            for (double x = clumpMinX; x <= clumpMaxX; x += fuelSpacing)
                for (double y = clumpMinY; y <= clumpMaxY; y += fuelSpacing)
                    fuel.add(new Translation2d(x, y));
    }

    private SimFuelClump() {}

    /** Called by GroundIntakeIOSim each loop with what the intake is being told to do. */
    public static void setIntakeCommand(double actuatorPositionInches, double rollerTargetVelocityRPS, double rollerFreeVelocityRPS) {
        actuatorInches = actuatorPositionInches;
        rollerTargetRPS = rollerTargetVelocityRPS;
        rollerFreeRPS = rollerFreeVelocityRPS;
    }

    /** Fraction of commanded roller speed the rollers reach under the current jam. */
    public static double getRollerSpeedFactor() {
        return MathUtil.clamp(1d - jam / jamStall, 0d, 1d);
    }

    /** Fraction of wheel motion that moves the robot. Below 1 when riding up on fuel. */
    public static double getTraction() {
        return 1d - 0.95d * MathUtil.clamp((jam - beachStart) / (beachFull - beachStart), 0d, 1d);
    }

    /** Robot tilt in degrees caused by riding up on the pile. */
    public static double getPitchDegrees() {
        return Math.min(15d, pitchPerFuel * Math.max(0d, jam - beachStart));
    }

    private static boolean intakeRunning() {
        return actuatorInches > deployedInches && rollerTargetRPS > 1d;
    }

    /** Call once per loop from simulationPeriodic. */
    public static void step(double dt) {
        if (!enabled || Drive.instance == null)
            return;

        final Pose2d pose = StateMachine.odometryAndVision.getEstimatedPose();
        final ChassisSpeeds speeds = Drive.instance.getChassisSpeeds();
        final boolean running = intakeRunning();

        // Speed toward the intake side, robot frame (positive = driving into fuel with the intake)
        final double intoIntake = intakeSide * speeds.vxMetersPerSecond;

        int arrived = 0;
        for (int i = fuel.size() - 1; i >= 0; i--) {
            Translation2d local = fuel.get(i).minus(pose.getTranslation()).rotateBy(pose.getRotation().unaryMinus());
            double x = local.getX() * intakeSide; // distance out the intake side
            double y = local.getY();

            boolean inMouth = running && x >= bumperHalf && x <= bumperHalf + mouthDepth && Math.abs(y) <= mouthHalfWidth;
            if (inMouth) {
                fuel.remove(i);
                arrived++;
                continue;
            }

            // Push fuel out of the bumper footprint (and the deployed intake's side plates)
            double reach = bumperHalf + 0.075d;
            double xMax = running ? bumperHalf + mouthDepth : reach;
            if (Math.abs(y) < reach && x > -reach && x < xMax) {
                double toSide = reach - Math.abs(y);
                double toFront = x > 0 ? xMax - x : x + reach;
                if (toSide < toFront)
                    y = Math.copySign(reach, y);
                else
                    x = x > 0 ? xMax : -reach;
                Translation2d moved = new Translation2d(x * intakeSide, y).rotateBy(pose.getRotation()).plus(pose.getTranslation());
                fuel.set(i, moved);
            }
        }
        jam += arrived;

        // Rollers pull jammed fuel in, slower as the jam grows
        final double rollerFraction = rollerFreeRPS > 0d ? Math.min(rollerTargetRPS, rollerFreeRPS) / rollerFreeRPS : 0d;
        if (running) {
            double eaten = Math.min(jam, ingestCapacity * rollerFraction * getRollerSpeedFactor() * dt);
            jam -= eaten;
            ingested += eaten;
        }

        // Backing away from the pile drops jammed fuel back on the field
        if (intoIntake < -0.05d && jam > 0d) {
            double released = Math.min(jam, releasePerMeter * -intoIntake * dt);
            jam -= released;
            releaseCarry += released;
            for (; releaseCarry >= 1d; releaseCarry -= 1d) {
                Translation2d local = new Translation2d((bumperHalf + mouthDepth + 0.1d) * intakeSide, (Math.random() - 0.5d) * 2d * mouthHalfWidth);
                fuel.add(local.rotateBy(pose.getRotation()).plus(pose.getTranslation()));
            }
        }

        Translation3d[] positions = new Translation3d[fuel.size()];
        for (int i = 0; i < positions.length; i++)
            positions[i] = new Translation3d(fuel.get(i).getX(), fuel.get(i).getY(), 0.075d);
        Logger.recordOutput("SimFuelClump/Fuel", positions);
        Logger.recordOutput("SimFuelClump/Jam", jam);
        Logger.recordOutput("SimFuelClump/Ingested", ingested);
        Logger.recordOutput("SimFuelClump/Traction", getTraction());
        Logger.recordOutput("SimFuelClump/PitchDegrees", getPitchDegrees());
        Logger.recordOutput("SimFuelClump/RollerSpeedFactor", getRollerSpeedFactor());
        Logger.recordOutput("SimFuelClump/IntakeRunning", running);
    }
}
