package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Follower follower;

    public Robot(HardwareMap hwMap) {
        follower = Constants.createFollower(hwMap);
    }
}
