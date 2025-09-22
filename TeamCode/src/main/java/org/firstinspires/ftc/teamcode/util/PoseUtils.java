package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class PoseUtils {

    public static Pose fromPose3d(Pose3D original) {
        return new Pose(
                original.getPosition().x,
                original.getPosition().y,
                original.getOrientation().getYaw()
        );
    }

//    public static
}
