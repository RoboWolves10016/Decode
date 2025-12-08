package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class FarAutonPoses {

    public static Pose startPose;
    public static Pose launchPose;

    public static Pose preCornerPose;
    public static Pose cornerPose;
    public static Pose preIntake1;
    public static Pose postIntake1;
    public static Pose preIntake2;
    public static Pose postIntake2;

    public static Pose endPose;

    public static PathChain startToLaunch;
    public static PathChain launchToRow1;
    public static PathChain row1ToLaunch;
    public static PathChain launchToCorner;
    public static PathChain cornerToLaunch;

    public static PathChain launchToRow2;
    public static PathChain row2ToLaunch;
    public static PathChain launchToEnd;


    public static void setAlliance(Alliance alliance) {
        startPose = new Pose(88, 8, Math.toRadians(90));
        launchPose = new Pose(86, 21, Math.toRadians(70));
        preCornerPose = new Pose(134.5, 26, Math.toRadians(-82));
        cornerPose = new Pose(134.5, 10, Math.toRadians(-82));
        preIntake1 = new Pose(96, 36, Math.toRadians(0));
        postIntake1 = new Pose(130, 36, Math.toRadians(0));
        preIntake2 = new Pose(96, 60, Math.toRadians(0));
        postIntake2 = new Pose(130, 60, Math.toRadians(0));
        endPose = new Pose(105, 17, Math.toRadians(0));

        if (alliance == Alliance.BLUE) {
            startPose = startPose.mirror();
            launchPose = launchPose.mirror();

            preCornerPose = preCornerPose.mirror();
            cornerPose = cornerPose.mirror();

            preIntake1 = preIntake1.mirror();
            postIntake1 = postIntake1.mirror();

            preIntake2 = preIntake2.mirror();
            postIntake2 = postIntake2.mirror();

            endPose = endPose.mirror();
        }
    }

    public static void createPaths(Follower follower, Alliance alliance) {
        startToLaunch = follower.pathBuilder()
                .addPath(
                        new BezierLine(startPose, launchPose)
                )
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

        launchToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, preIntake1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), preIntake1.getHeading())
                .addPath(new BezierLine(preIntake1, postIntake1))
                .setLinearHeadingInterpolation(preIntake1.getHeading(), postIntake1.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(0.3))
                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
                .build();


        row1ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(postIntake1, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

        launchToCorner = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, preCornerPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), preCornerPose.getHeading())
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.5))
                .addPath(new BezierLine(preCornerPose, cornerPose))
                .setLinearHeadingInterpolation(preCornerPose.getHeading(), cornerPose.getHeading())
                .addParametricCallback(0.3, () -> follower.setMaxPower(0.2))
                .build();

        cornerToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.7,
                                HeadingInterpolator.linear(cornerPose.getHeading(), launchPose.getHeading())),
                        new HeadingInterpolator.PiecewiseNode(
                                0.7,
                                1.0,
                                HeadingInterpolator.facingPoint(alliance.goalPose)
                        )
                )).build();

        launchToRow2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, preIntake2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), preIntake2.getHeading())
                .addPath(new BezierLine(preIntake2, postIntake2))
                .setLinearHeadingInterpolation(preIntake1.getHeading(), postIntake1.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(0.3))
                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
                .build();

        row2ToLaunch = follower
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
