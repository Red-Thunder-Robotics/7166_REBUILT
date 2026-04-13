// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ApriltagUtil;

public final class Constants {
    public static final Mode SIM_MODE = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    // NOTE: THERE ARE OVERLAPPING CONTROLS!!!!
    private static final boolean USE_TWO_CONTROLLERS_DESIRED = false;
    public static final boolean USE_TWO_CONTROLLERS = USE_TWO_CONTROLLERS_DESIRED;// ? RobotBase.isSimulation() : false;
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = USE_TWO_CONTROLLERS ? DRIVER_CONTROLLER : 1;

    public static final CANBus CANBUS = new CANBus("7166CANivore");

    public static final boolean USE_TURRET = false;
    public static final boolean USE_MACKINAC = false;

    public static final class FieldConstants {
        public static final Distance APRIL_TAG_WIDTH = Inches.of(6.5d);

        public static final Distance FUNNEL_RADIUS = Inches.of(24d);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72d - 56.4d);

        public static final Distance FIELD_LENGTH = Meters.of(ApriltagUtil.fieldLayout.getFieldLength());
        public static final Distance FIELD_WIDTH = Meters.of(ApriltagUtil.fieldLayout.getFieldWidth());

        public static final Distance FUEL_DIAMETER = Centimeters.of(15d);
    }

    public static final class DriveConstants {
        public static final double TRANSLATION_KP = 5.5d;
        public static final boolean ANGLE_LIVE_TUNE = false;
        public static final double ANGLE_KP = 0.5d;
        public static final double ANGLE_KI = 0d;
        public static final double ANGLE_KD = 0d;
        
        // degrees
        public static final double IS_FACING_ALLIANCE_ZONE_THRESHOLD = 25; // 22
        // meters per second
        public static final double ALLIANCE_FEED_MAX_SPEEDY = 2d;

        // public static final Distance BUMPER_WIDTH = Inches.of(34.52d); // 31
        public static final Distance BUMPER_WIDTH = Millimeters.of(880d); // FIXME: should be 840
        public static final Distance BUMPER_LENGTH = BUMPER_WIDTH;

        // at meters min distance, degrees min threshold will be used, etc
        public static final double SHOULD_INDEX_THRESHOLD_METERS_MIN = 1d;
        public static final double SHOULD_INDEX_THRESHOLD_DEGREES_MIN = 5d;
        public static final double SHOULD_INDEX_THRESHOLD_METERS_MAX = 5.3d;
        public static final double SHOULD_INDEX_THRESHOLD_DEGREES_MAX = 1d;

        // NOTE: below is not used
        // public static final double SHOULD_INDEX_THRESHOLD_MOVING_DEGREES = 3d;
        public static final double IS_MOVING_THRESHOLD_METERS = Units.inchesToMeters(2d);

        public static final boolean DRIVE_HIGHER_AUTO_LIMITS = false;

        public static final double DRIVE_STATOR_LOW = TunerConstants.kSlipCurrent.in(Amps);
        public static final double DRIVE_STATOR_AUTO = DRIVE_HIGHER_AUTO_LIMITS ? Math.max(DRIVE_STATOR_LOW, 200) : DRIVE_STATOR_LOW;
        public static final double DRIVE_STATOR_TURBO = Math.max(DRIVE_STATOR_LOW, 160);

        public static final double DRIVE_SUPPLY_LOW = 50;
        public static final double DRIVE_SUPPLY_AUTO = DRIVE_HIGHER_AUTO_LIMITS ? Math.max(DRIVE_SUPPLY_LOW, 70) : DRIVE_SUPPLY_LOW;
        public static final double DRIVE_SUPPLY_TURBO = Math.max(DRIVE_SUPPLY_LOW, 60);
    }
}
