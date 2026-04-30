package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class CloseAutonPaths {

    // Poses are RED by default, mirror for blue
    public static Pose startingPose = new Pose(14, 114.5, Math.toRadians(0));
    public static Pose launchPose = new Pose(50, 90, Math.toRadians(0));

    // Poses for intaking from row 3
    public static Pose row1Control = new Pose();
    public static Pose row1End = new Pose();
    public static Pose row2Control = new Pose();
    public static Pose row2End = new Pose();


    /** FAR AUTON PATHS **/
    public static PathChain startToLaunch;
    public static PathChain launchToRow1;
    public static PathChain row1ToLaunch;
//    public static PathChain row1ToDump;
//    public static PathChain launchToDump;
//    public static PathChain dumpToLaunch;
    public static PathChain launchToRow2;
    public static PathChain row2ToLaunch;
//    public static PathChain launchToCatch;
//    public static PathChain catchToLaunch;
    public static void setAlliance(Alliance alliance) {
        if (alliance == Alliance.RED) {
            startingPose = startingPose.mirror();
            launchPose = launchPose.mirror();
            row1Control = row1Control.mirror();
            row1End = row1End.mirror();
            row2Control = row2Control.mirror();
            row2End = row2End.mirror();
        }
    }

    public static void createPaths(Follower follower) {
        startToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(
                        startingPose,
                        launchPose
                )).setLinearHeadingInterpolation(startingPose.getHeading(), launchPose.getHeading())
                .build();

        launchToRow1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                                launchPose,
                                row1Control,
                                row1End)
                ).setTangentHeadingInterpolation()
                .build();

        row1ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(row1End, launchPose))

                .build();

//        row1ToDump = follower.pathBuilder()
//                .addPath(new BezierCurve(row1End)).build();

        row2ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(
                        row2End,
                        launchPose)
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.8, HeadingInterpolator.tangent),
                        new HeadingInterpolator.PiecewiseNode(0.8, 1.0, HeadingInterpolator.constant(launchPose.getHeading()))
                )).build();

    }
}
