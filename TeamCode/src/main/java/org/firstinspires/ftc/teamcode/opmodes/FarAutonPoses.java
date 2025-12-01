package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class FarAutonPoses {

    public static Pose startPose;
    public static Pose launchPose;

    public static Pose preIntake1;
    public static Pose postIntake1;
    public static Pose preIntake2;
    public static Pose postIntake2;

    public static Pose endPose;

    public static PathChain startToLaunch;
    public static PathChain launchToRow1;
    public static PathChain row1ToLaunch;

    public static PathChain launchToRow2;
    public static PathChain row2ToLaunch;
    public static PathChain launchToEnd;


    public static void setAlliance(Alliance alliance) {
        startPose = new Pose(88, 9, Math.toRadians(90));
        launchPose = new Pose(86, 21, Math.toRadians(50));
        preIntake1 = new Pose(96, 36, Math.toRadians(0));
        postIntake1 = new Pose(130, 36, Math.toRadians(0));
        preIntake2 = new Pose(96, 60, Math.toRadians(0));
        postIntake2 = new Pose(130, 60, Math.toRadians(0));
        endPose = new Pose(105, 17, Math.toRadians(0));

        if (alliance == Alliance.BLUE) {
            startPose = startPose.mirror();
            launchPose = launchPose.mirror();

            preIntake1 = preIntake1.mirror();
            postIntake1 = postIntake1.mirror();

            preIntake2 = preIntake2.mirror();
            postIntake2 = postIntake2.mirror();

            endPose = endPose.mirror();
        }
    }

    public static void createPaths(Follower follower, Alliance alliance) {
        startToLaunch = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, launchPose)
                )
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

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

        launchToRow2 = follower
                .pathBuilder()
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
