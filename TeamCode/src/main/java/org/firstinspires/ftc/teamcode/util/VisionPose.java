package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class VisionPose {
    public Pose pose;
    public long timestampNs;
    public VisionPose(Pose pose, long timestampNs) {
        this.pose = pose;
        this.timestampNs = timestampNs;
    }
}
