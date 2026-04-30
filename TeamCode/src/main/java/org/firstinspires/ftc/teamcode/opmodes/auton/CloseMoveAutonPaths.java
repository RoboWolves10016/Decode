
package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class CloseMoveAutonPaths {

    // Poses are BLUE by default, mirror for blue
    public static Pose startingPose = new Pose(32, 136.5, Math.toRadians(0));
    public static Pose launchPose = new Pose(57, 124, Math.toRadians(0));

    public static PathChain startToLaunch;

    public static void setAlliance(Alliance alliance) {
        if (alliance == Alliance.RED) {
            startingPose = startingPose.mirror();
            launchPose = launchPose.mirror();
        }
    }

    public static void createPaths(Follower follower) {
        startToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, launchPose))
                .setConstantHeadingInterpolation(0)
                .build();
    }
}
