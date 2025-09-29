package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class PoseUtils {

    public static final double FIELD_WIDTH_METERS = 3.6576;

    public static Pose fromPose3d(Pose3D original) {
        return new Pose(
                original.getPosition().x,
                original.getPosition().y,
                original.getOrientation().getYaw(AngleUnit.RADIANS),
                InvertedFTCCoordinates.INSTANCE
        ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
    public static boolean isInField(Pose pose) {
        return Math.abs(pose.getX())< FIELD_WIDTH_METERS / 2
                && Math.abs(pose.getY()) < FIELD_WIDTH_METERS / 2;
    }

    public static String poseToString(Pose pose) {
        return "("
                + truncate(pose.getX(), 3) + ", "
                + truncate(pose.getY(), 3) + ", "
                + truncate(Math.toDegrees(pose.getHeading()), 3) + ")";
    }

    public static double truncate(double d, int decimals) {
        double precision = Math.pow(10, decimals);
        if (d > 0) {
            return Math.floor(d * precision) / precision;
        } else {
            return Math.ceil(d * precision) / precision;
        }
    }
}
