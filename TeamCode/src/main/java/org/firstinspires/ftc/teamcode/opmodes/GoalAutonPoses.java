package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class GoalAutonPoses {

    public static Pose startPose;
    public static Pose launchPose;
//    public static Pose row3Control;
    public static Pose preIntake3;
    public static Pose postIntake3;
    public static Pose row2Control;
    public static Pose preIntake2;
    public static Pose postIntake2;
//    public static Pose row1Control;
    public static Pose preIntake1;
    public static Pose postIntake1;
    public static Pose endPose;

    public static PathChain startToLaunch;
    public static PathChain launchToRow3;
    public static PathChain row3ToLaunch;
    public static PathChain launchToRow2;
    public static PathChain row2ToLaunch;
    public static PathChain launchToRow1;
    public static PathChain row1ToLaunch;
    public static PathChain launchToEnd;


    public static void setAlliance(Alliance alliance) {
        startPose = new Pose(127.5, 124.5, Math.toRadians(35.3));
        launchPose = new Pose(96, 96, Math.toRadians(50));
        preIntake3 = new Pose(96, 84, Math.toRadians(0));
//        row3Control = new Pose(86, 80);
        postIntake3 = new Pose(127, 84, Math.toRadians(0));
        preIntake2 = new Pose(96, 60, Math.toRadians(0));
        postIntake2 = new Pose(130, 60, Math.toRadians(0));
        row2Control = new Pose(95, 60);
//        row1Control = new Pose(75, 30);
        preIntake1 = new Pose(96, 36, Math.toRadians(0));
        postIntake1 = new Pose(130, 36, Math.toRadians(0));
        endPose = new Pose(100, 72, Math.toRadians(0));

        if (alliance == Alliance.BLUE) {
            startPose = startPose.mirror();
            launchPose = launchPose.mirror();

//            row3Control = row3Control.mirror();
            preIntake3 = preIntake3.mirror();
            postIntake3 = postIntake3.mirror();

            row2Control = row2Control.mirror();
            preIntake2 = preIntake2.mirror();
            postIntake2 = postIntake2.mirror();

//            row1Control = row1Control.mirror();
            preIntake1 = preIntake1.mirror();
            postIntake1 = postIntake1.mirror();

            endPose = endPose.mirror();
        }
    }

    public static void createPaths(Follower follower, Alliance alliance) {
        startToLaunch = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(GoalAutonPoses.startPose, GoalAutonPoses.launchPose)
                )
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

//        launchToRow3 = follower
//                .pathBuilder()
//                .addPath(new BezierCurve(launchPose, row3Control, postIntake3))
////                .setLinearHeadingInterpolation(launchPose.getHeading(), postIntake3.getHeading())
//                .setHeadingInterpolation(HeadingInterpolator.piecewise(
//                        new HeadingInterpolator.PiecewiseNode(
//                                0.0,
//                                0.2,
//                                HeadingInterpolator.linear(launchPose.getHeading(), postIntake3.getHeading())),
//                        new HeadingInterpolator.PiecewiseNode(
//                                0.2,
//                                1.0,
//                                HeadingInterpolator.constant(postIntake3.getHeading())
//                        )
//                ))
//                .addParametricCallback(0.2, () -> follower.setMaxPower(0.3))
//                .build();

        launchToRow3 = follower
                .pathBuilder()
                .addPath(new BezierLine(launchPose, preIntake3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), preIntake3.getHeading())
                .addPath(new BezierLine(preIntake3, postIntake3))
                .setLinearHeadingInterpolation(preIntake3.getHeading(), postIntake3.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(0.3))
                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
                .build();

        row3ToLaunch = follower
                .pathBuilder()
                .addPath(new BezierLine(postIntake3, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

//        launchToRow2 = follower
//                .pathBuilder()
//                .addPath(new BezierCurve(launchPose, row2Control, postIntake2))
//                .setLinearHeadingInterpolation(launchPose.getHeading(), postIntake2.getHeading())
//                .setHeadingInterpolation(HeadingInterpolator.piecewise(
//                        new HeadingInterpolator.PiecewiseNode(
//                                0.0,
//                                0.5,
//                                HeadingInterpolator.linear(launchPose.getHeading(), postIntake2.getHeading())),
//                        new HeadingInterpolator.PiecewiseNode(
//                                0.5,
//                                1.0,
//                                HeadingInterpolator.constant(postIntake3.getHeading())
//                        )
//                ))
//                .addParametricCallback(0.5, () -> follower.setMaxPower(0.3))
//                .build();

        launchToRow2 = follower
                .pathBuilder()
                .addPath(new BezierLine(launchPose, preIntake2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), preIntake2.getHeading())
                .addPath(new BezierLine(preIntake2, postIntake2))
                .setLinearHeadingInterpolation(preIntake2.getHeading(), postIntake2.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(0.3))
                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
                .build();


        row2ToLaunch = follower
                .pathBuilder()
                .addPath(new BezierCurve(postIntake2, row2Control, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

//        launchToRow1 = follower
//                .pathBuilder()
//                .addPath(new BezierCurve(launchPose, row1Control, postIntake1))
//                .addParametricCallback(0.5, () -> follower.setMaxPower(0.4))
//                .setHeadingInterpolation(HeadingInterpolator.piecewise(
//                        new HeadingInterpolator.PiecewiseNode(
//                                0.0,
//                                0.5,
//                                HeadingInterpolator.linear(launchPose.getHeading(), postIntake1.getHeading())),
//                        new HeadingInterpolator.PiecewiseNode(
//                                0.5,
//                                1.0,
//                                HeadingInterpolator.constant(postIntake3.getHeading())
//                        )
//                ))
//                .build();

        launchToRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(launchPose, preIntake1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), preIntake1.getHeading())
                .addPath(new BezierLine(preIntake1, postIntake1))
                .setLinearHeadingInterpolation(preIntake1.getHeading(), postIntake1.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(0.3))
                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
                .build();


        row1ToLaunch = follower
                .pathBuilder()
                .addPath(new BezierLine(postIntake1, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

        launchToEnd = follower
                .pathBuilder()
                .addPath(new BezierLine(launchPose, endPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), endPose.getHeading())
                .build();
    }
}
