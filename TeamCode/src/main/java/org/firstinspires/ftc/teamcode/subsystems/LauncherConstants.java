package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LauncherConstants {
    // Flywheel Constants
    public static final double[] SHOT_DISTANCES = {24, 32, 40, 52, 64, 75, 85, 95, 110, 130, 140, 150};
    public static final double[] SHOT_SPEEDS = {2600, 2600, 2700, 2800, 3000, 3050, 3200, 3350, 3500, 3850, 4000, 4075};
    public static final double[] AIR_TIMES = {0.5, 0.5, 0.5, 0.6, 0.75, 0.8, 0.85, 1.0, 1.1, 1.2};

    // Turret Constants
    public static final double RIGHT_TURRET_POS = 0.251;
    public static final double RIGHT_TURRET_ANGLE = -90;
    public static final double LEFT_TURRET_POS = 0.723;
    public static final double LEFT_TURRET_ANGLE = 90;
    public static final double MAX_TURRET_ANGLE_LIMIT = 120;
    public static final double MIN_TURRET_ANGLE_LIMIT = -120;
//    public static final double MAX_TURRET_ANGLE_LIMIT = 135;
//    public static final double MIN_TURRET_ANGLE_LIMIT = -135;

    public static final double TURRET_ROT_FF = 0.15;

    public static final double LEFT_WRAPAROUND_POINT = 30;
    public static final double RIGHT_WRAPAROUND_POINT = -30;

    public static final double TURRET_CACHING_TOL_DEG = 0.5;

    // Hood Constants
//    public static final double[] SHOT_ANGLES = {22, 25, 26, 30, 32, 35, 37, 39, 43, 45, 47};
    public static final double[] SHOT_ANGLES = {22, 25, 26, 30, 32, 38, 39.5, 41, 43, 45, 47, 47};
    public static final double BOTTOM_HOOD_POS = 0.92;
    public static final double BOTTOM_HOOD_ANGLE = 22d;
    public static final double TOP_HOOD_POS = 0.08;
    public static final double TOP_HOOD_ANGLE = 47d;
    public static final double HOOD_CACHING_TOL_DEG = 1d;
}
