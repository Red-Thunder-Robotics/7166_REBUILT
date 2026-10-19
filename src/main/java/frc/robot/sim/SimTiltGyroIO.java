package frc.robot.sim;

import frc.robot.subsystems.drive.GyroIO;

/**
 * Sim gyro that only reports tilt from SimFuelClump. It stays "disconnected"
 * so Drive keeps using wheel odometry for yaw, as it did with the empty GyroIO.
 */
public final class SimTiltGyroIO implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = false;
        // The intake is on the back, so riding up on fuel raises the back (front down). This assumes the
        // Pigeon 2 reports that as negative pitch (positive = nose up). Check on the real robot by tipping it.
        inputs.pitchRad = -Math.toRadians(SimFuelClump.getPitchDegrees());
        inputs.rollRad = 0d;
    }
}
