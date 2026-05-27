package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.PoseUtils.toInches;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

public enum Alliance {
    RED(
            0,
//            new Pose(144 - 5, 144 - 5),
            new Pose(144 - 5, 144 - 3)
    ),
    BLUE(
            Math.PI,
//            new Pose(5, 144 - 5),
            new Pose(10, 144 - 10)
    );

    public final double driverForwardHeading;
    public final Pose goalPose;
//    public final Pose aimPose;

    Alliance(double forwardDirection, Pose goalPose) {
        this.driverForwardHeading = forwardDirection;
        this.goalPose = goalPose;
//        this.aimPose = aimPose;
    }
}
