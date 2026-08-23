package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.state_machine.StateMachine;
import org.littletonrobotics.junction.Logger;

// https://github.com/CrossTheRoadElec/2026-Rebuilt-CTR-Example-Robot/blob/main/src/main/java/frc/robot/utils/HubActiveState.java

public class HubActiveState {
    private static class TimeSegment {
        public double start;
        public double end;

        public TimeSegment(double start, double end) {
            this.start = start;
            this.end = end;
        }

        public boolean isTimeWithin(double time) {
            return time >= start && time <= end;
        }
    }

    private static final TimeSegment[] TeleopHubActiveTimesForAutoWinner = new TimeSegment[] {
        new TimeSegment(0, 55), // Shift 4 combines with End game
        new TimeSegment((1 * 60) + 20, (1 * 60) + 45), // Shift 2
        new TimeSegment((2 * 60) + 10, (2 * 60) + 20) // Transition Shift
    };
    private static final TimeSegment[] TeleopHubActiveTimesForAutoLoser = new TimeSegment[] {
        new TimeSegment(0, 30), // End game
        new TimeSegment(55, (1 * 60) + 20), // Shift 3
        new TimeSegment((1 * 60) + 45, (2 * 60) + 20) // Shift 1 combines with transition shift
    };

    private static boolean isOurHubActive = false;
    private static double timeUntilSwap = 0;

    private static void updateStatesForTeleop() {
        if (!DriverStation.isTeleopEnabled()) return;

        final String message = DriverStation.getGameSpecificMessage();
        final boolean redIsWinner = message.equals("R");

        final boolean isRedAlliance = StateMachine.ALLIANCE == Alliance.Red;
        final boolean useWinnerTimes = redIsWinner == isRedAlliance;

        final double timeLeftInTeleop = DriverStation.getMatchTime();
        double newTimeUntilSwap = 150;

        for (TimeSegment seg : useWinnerTimes ? TeleopHubActiveTimesForAutoWinner : TeleopHubActiveTimesForAutoLoser) {
            if (seg.isTimeWithin(timeLeftInTeleop)) {
                timeUntilSwap = timeLeftInTeleop - seg.start;
                isOurHubActive = true;
                return;
            }
            double timeToStart = timeLeftInTeleop - seg.end;
            if (timeToStart > 0 && timeToStart < newTimeUntilSwap) newTimeUntilSwap = timeToStart;
        }
        timeUntilSwap = newTimeUntilSwap;
        isOurHubActive = false;
    }

    public static void periodic() {
        if (!Logger.hasReplaySource()) {
            if (DriverStation.isDisabled()) {
                /* If we're disabled, the hub is always inactive */
                isOurHubActive = false;
                timeUntilSwap = -1;
            } else if (DriverStation.isAutonomous()) {
                /* If we're autonomous, the hub is always active */
                isOurHubActive = true;
                timeUntilSwap = DriverStation.getMatchTime();
            } else if (DriverStation.isTeleop()) updateStatesForTeleop();
            else isOurHubActive = false;
        }
    }

    public static boolean isOurHubActive() {
        return isOurHubActive;
    }

    public static double timeUntilSwap() {
        return timeUntilSwap;
    }
}
