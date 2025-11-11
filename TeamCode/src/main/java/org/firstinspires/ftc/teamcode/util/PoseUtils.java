package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.pedropathing.Tuning.Drawing;

public class PoseUtils {

    public static final double FIELD_WIDTH_INCHES = 144;

    public static Pose fromPose3d(Pose3D original) {
        Pose pose = new Pose(
                toInches(original.getPosition().y) + 72,
                -toInches(original.getPosition().x) + 72,
                original.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI / 2
        );
        return pose.setHeading((pose.getHeading() + Math.PI) % (2 * Math.PI));
    }
//
//    public static void drawOnlyCurrent(Follower follower) {
//        try {
//            Drawing.drawRobot(follower.getPose());
//            Drawing.sendPacket();
//        } catch (Exception e) {
//            throw new RuntimeException("Drawing failed " + e);
//        }
//    }
//
//    public static void draw(Follower follower) {
//        Drawing.drawDebug(follower);
//    }

    public static boolean isInField(Pose pose) {
        return pose.getX() > 0 && pose.getX() < FIELD_WIDTH_INCHES
                && pose.getY() > 0 && pose.getY() < FIELD_WIDTH_INCHES;
    }

    public static String poseToString(Pose pose) {
        return "("
                + truncate(pose.getX(), 3) + ", "
                + truncate(pose.getY(), 3) + ", "
                + truncate(Math.toDegrees(pose.getHeading()), 3) + ")";
    }

    public static String vectorToString(Vector vector) {
        return "Vector{"
                + truncate(vector.getXComponent(), 3) + ", "
                + truncate(vector.getYComponent(), 3) + ", "
                + "}";
    }

    public static double truncate(double d, int decimals) {
        double precision = Math.pow(10, decimals);
        if (d > 0) {
            return Math.floor(d * precision) / precision;
        } else {
            return Math.ceil(d * precision) / precision;
        }
    }

    // Returns the given heading bounded between -PI and +PI
    public static double normalizeHeading(double heading) {
        while (heading < -Math.PI) {
            heading += 2 * Math.PI;
        }
        return heading;
    }

    public static double toInches(double meters) {
        return meters * 100 / 2.54;
    }
}
