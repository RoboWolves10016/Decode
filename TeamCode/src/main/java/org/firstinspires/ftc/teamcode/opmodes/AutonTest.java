package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name="AutonTest")
@Disabled
public class AutonTest extends OpMode {
    Follower follower = Constants.createFollower(hardwareMap);
    PathChain pathChain;

    @Override
    public void init() {
        pathChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(0,0,0), new Pose(20, 0, 100)))
                .build();
    }

    @Override
    public void loop() {
        follower.followPath(pathChain);
    }
}
