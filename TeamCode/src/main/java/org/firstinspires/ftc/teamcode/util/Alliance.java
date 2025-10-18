package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public enum Alliance {
    RED(Math.PI/2, new Pose(0.5877852522924731,0.8090169943749475)),
    BLUE(-Math.PI/2, new Pose(0.5877852522924731, -0.8090169943749475));

    public final double driverForwardHeading;
    public final Pose goalPose;

    Alliance(double forwardDirection, Pose goalPose) {
        this.driverForwardHeading = forwardDirection;
        this.goalPose = goalPose;
    }
}
