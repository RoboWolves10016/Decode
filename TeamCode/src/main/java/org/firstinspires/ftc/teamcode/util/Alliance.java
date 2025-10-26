package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.PoseUtils.toInches;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

public enum Alliance {
    RED(
            -Math.PI/2,
            new Pose(
                    144 - 14,
                    144 - 13,
                    0,
                    PedroCoordinates.INSTANCE
            )),
    BLUE(
            Math.PI / 2,
            new Pose(
                    14,
                    144 - 13,
                    0,
                    PedroCoordinates.INSTANCE
            ));

    public final double driverForwardHeading;
    public final Pose goalPose;

    Alliance(double forwardDirection, Pose goalPose) {
        this.driverForwardHeading = forwardDirection;
        this.goalPose = goalPose;
    }
}
