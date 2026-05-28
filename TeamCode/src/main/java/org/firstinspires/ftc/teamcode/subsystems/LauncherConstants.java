package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LauncherConstants {
    // Indexing Constants
    public static double INDEX_TIME = 0.5;
    // Flywheel Constants
//    public static final double[] SHOT_DISTANCES = {24, 32, 40, 52, 64, 75, 85, 95, 110, 130, 140, 150};
//    public static final double[] SHOT_SPEEDS = {2600, 2600, 2700, 2800, 3000, 3050, 3200, 3350, 3500, 3850, 4000, 4150};
    public static final double[] AIR_TIMES = {0.5, 0.5, 0.5, 0.6, 0.75, 0.8, 0.85, 1.0, 1.1, 1.2};

    public static final double[] SHOT_DISTANCES = {24, 32, 40, 52, 64, 94, 101, 119, 129, 140, 157};
    public static final double[] SHOT_SPEEDS = {2600, 2600, 2700, 2800, 3100, 3280, 3560, 3860, 4075, 4150, 4400};

    // Turret Constants
    public static final double RIGHT_TURRET_POS = 0.258;
    public static final double RIGHT_TURRET_ANGLE = -90;
    public static final double LEFT_TURRET_POS = 0.751;
    public static final double LEFT_TURRET_ANGLE = 90;
    public static final double MIDDLE_TURRET_POS = (RIGHT_TURRET_POS + LEFT_TURRET_POS) / 2;
    public static final double MAX_TURRET_ANGLE_LIMIT = 150;
    public static final double MIN_TURRET_ANGLE_LIMIT = -135;
//    public static final double MAX_TURRET_ANGLE_LIMIT = 135;
//    public static final double MIN_TURRET_ANGLE_LIMIT = -135;

    public static final double LEFT_ANALOG_VALUE = 96.26;
    public static final double RIGHT_ANALOG_VALUE = 264.37;

    public static double TURRET_ROT_KV = 0.1;
//    public static double TURRET_ROT_KA = 0.0;

    public static final double LEFT_WRAPAROUND_POINT = 30;
    public static final double RIGHT_WRAPAROUND_POINT = -30;

    public static final double TURRET_CACHING_TOL_DEG = 0.5;

    // Hood Constants
//    public static final double[] SHOT_ANGLES = {22, 25, 26, 30, 32, 38, 39.5, 41, 43, 45, 47, 47};
    public static final double[] SHOT_ANGLES = {22, 25, 26, 30, 43, 45, 48, 50, 52, 52, 54};
    public static final double BOTTOM_HOOD_POS = 0.985;
//    public static final double BOTTOM_HOOD_ANGLE = 22d;
    public static final double BOTTOM_HOOD_ANGLE = 32d;
    public static final double TOP_HOOD_POS = 0.116;
//    public static final double TOP_HOOD_ANGLE = 47d;
    public static final double TOP_HOOD_ANGLE = 57d;
    public static final double HOOD_CACHING_TOL_DEG = 1d;

    // PRESET SHOT CONSTANTS
//    public static final Pose RED_SIDE_PRESET_POSE = new Pose(90, 7.44);
//    public static final Pose BLUE_SIDE_PRESET_POSE = new Pose(44, 7.44);

    public static final double PRESET_DEG_CLOSE_RED = 67;
    public static final double PRESET_DEG_FAR_RED = 67;
    public static final double PRESET_DEG_CLOSE_BLUE = 113;
    public static final double PRESET_DEG_FAR_BLUE = 113;

    public static final double PRESET_RPM_CLOSE = 3950;
    public static final double PRESET_RPM_FAR = 4500;
}
