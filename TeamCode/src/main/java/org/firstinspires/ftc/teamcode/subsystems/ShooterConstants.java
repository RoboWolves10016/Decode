package org.firstinspires.ftc.teamcode.subsystems;

public class ShooterConstants {
    // Turret Constants
    public static final double CENTER_POS = 0.5;
    public static final double CENTER_ANGLE = 0;
    public static final double MAX_ANGLE_LIMIT = CENTER_ANGLE + (3 * Math.PI / 4);
    public static final double MIN_ANGLE_LIMIT = CENTER_ANGLE - (3 * Math.PI / 4);
    public static final double TURRET_RANGE = 2 * Math.PI;
    public static final double TURRET_CACHING_TOL_DEG = 0.5;

    // Hood Constants
    public static final double BOTTOM_HOOD_POS = 1.0;
    public static final double TOP_HOOD_POS = 0.25;
    public static final double BOTTOM_HOOD_ANGLE = 22d;
    public static final double TOP_HOOD_ANGLE = 47d;
    public static final double HOOD_CACHING_TOL_DEG = 1d;
}
