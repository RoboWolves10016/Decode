package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class GoalAutonPoses {

    public static Pose obeliskPose = new Pose(72, 144);

    public static Pose straightStartPose;
    public static Pose sidewaysStartPose;
    public static Pose launchPose;
    public static Pose endLaunchPose;

    public static Pose dumpPose;
    public static Pose row3DumpControl;
    public static Pose preIntake3;
    public static Pose row3Control;
    public static Pose postIntake3;
    public static Pose afterRow2Control;
    public static Pose preIntake2;
    public static Pose row2Control;
    public static Pose postIntake2;
    public static Pose preIntake1;
    public static Pose row1Control;
    public static Pose postIntake1;
    public static Pose endPose;

    public static PathChain startToLaunch;
    public static PathChain startToLaunchObelisk;
    public static PathChain launchToRow3;
    public static PathChain row3ToLaunch;
//    public static PathChain row3ToDumpToLaunch;
    public static PathChain row3ToDump;
    public static PathChain dumpToLaunch;

    public static PathChain launchToRow2;
    public static PathChain row2ToLaunch;
    public static PathChain launchToRow1;
    public static PathChain row1ToLaunch;
    public static PathChain launchToEnd;


    public static void setAlliance(Alliance alliance) {
        straightStartPose = new Pose(127.5, 124.5, Math.toRadians(35.3));

        sidewaysStartPose = new Pose(126, 123, Math.toRadians(125.3));
        launchPose = new Pose(96, 96, Math.toRadians(50));
        endLaunchPose = new Pose(89, 105);
        dumpPose = new Pose(129, 75, Math.toRadians(0));

        row3DumpControl = new Pose(112, 76);
        preIntake3 = new Pose(96, 84, Math.toRadians(0));
        row3Control = new Pose(98, 83);
        postIntake3 = new Pose(127, 84, Math.toRadians(0));

        preIntake2 = new Pose(96, 60, Math.toRadians(0));
        row2Control = new Pose(72, 53);
        postIntake2 = new Pose(130, 60, Math.toRadians(0));
        afterRow2Control = new Pose(95, 60);

        preIntake1 = new Pose(96, 36, Math.toRadians(0));
//        row1Control = new Pose(78, 30);
        row1Control = new Pose(68, 27);
        postIntake1 = new Pose(130, 36, Math.toRadians(0));
        endPose = new Pose(100, 72, Math.toRadians(0));

        if (alliance == Alliance.BLUE) {
            straightStartPose = straightStartPose.mirror();
            sidewaysStartPose = sidewaysStartPose.mirror();
            launchPose = launchPose.mirror();
            endLaunchPose = endLaunchPose.mirror();
            dumpPose = dumpPose.mirror();
            row3DumpControl = row3DumpControl.mirror();

            preIntake3 = preIntake3.mirror();
            row3Control = row3Control.mirror();
            postIntake3 = postIntake3.mirror();

            preIntake2 = preIntake2.mirror();
            row2Control = row2Control.mirror();
            postIntake2 = postIntake2.mirror();
            afterRow2Control = afterRow2Control.mirror();

            preIntake1 = preIntake1.mirror();
            row1Control = row1Control.mirror();
            postIntake1 = postIntake1.mirror();

            endPose = endPose.mirror();
        }
    }

    public static void createPaths(Follower follower, Alliance alliance) {
        startToLaunch = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(GoalAutonPoses.straightStartPose, GoalAutonPoses.launchPose)
                )
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

        startToLaunchObelisk = follower.pathBuilder()
                .addPath(new BezierLine(sidewaysStartPose, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.7, HeadingInterpolator.facingPoint(obeliskPose)),
                        new HeadingInterpolator.PiecewiseNode(0.7, 1.0, HeadingInterpolator.facingPoint(alliance.goalPose))
                ))
                .build();

//        launchToRow3 = follower
//                .pathBuilder()
//                .addPath(new BezierLine(launchPose, preIntake3))
//                .setLinearHeadingInterpolation(launchPose.getHeading(), preIntake3.getHeading())
//                .addPath(new BezierLine(preIntake3, postIntake3))
//                .setLinearHeadingInterpolation(preIntake3.getHeading(), postIntake3.getHeading())
//                .addParametricCallback(0, () -> follower.setMaxPower(0.3))
//                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
//                .build();
        launchToRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, row3Control, postIntake3))
                .setConstantHeadingInterpolation(postIntake3.getHeading())
                .addParametricCallback(0.2, () -> follower.setMaxPower(0.3))
                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
                .build();

        row3ToLaunch = follower
                .pathBuilder()
                .addPath(new BezierLine(postIntake3, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

//        row3ToDumpToLaunch = follower
//                .pathBuilder()
//                .addPath(new BezierCurve(postIntake3, row3DumpControl, dumpPose))
//                .setLinearHeadingInterpolation(postIntake3.getHeading(), dumpPose.getHeading())
//                .addPath(new BezierLine(dumpPose,launchPose))
//                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
//                .build();

        row3ToDump = follower.pathBuilder()
                .addPath(new BezierCurve(postIntake3, row3DumpControl, dumpPose))
                .setConstantHeadingInterpolation(dumpPose.getHeading())
                .addParametricCallback(0.0, () -> follower.setMaxPower(0.7))
                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
                .build();

        dumpToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(dumpPose, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.3, HeadingInterpolator.constant(dumpPose.getHeading())),
                        new HeadingInterpolator.PiecewiseNode(0.3, 1.0, HeadingInterpolator.facingPoint(alliance.goalPose))
                )).build();


//        row3ToDumpToLaunch = follower.pathBuilder()
//                .addPath(new BezierCurve(postIntake3, row3DumpControl, dumpPose))
//                .setConstantHeadingInterpolation(dumpPose.getHeading())
//                .addParametricCallback(0.0, () -> follower.setMaxPower(0.7))
//                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
//                .addPath(new BezierLine(dumpPose, launchPose))
//                .setHeadingInterpolation(HeadingInterpolator.piecewise(
//                        new HeadingInterpolator.PiecewiseNode(0.0, 0.3, HeadingInterpolator.constant(dumpPose.getHeading())),
//                        new HeadingInterpolator.PiecewiseNode(0.3, 1.0, HeadingInterpolator.facingPoint(alliance.goalPose))
//                )).build();

        launchToRow2 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, row2Control, postIntake2))
                .setConstantHeadingInterpolation(postIntake2.getHeading())
                .addParametricCallback(0.4, () -> follower.setMaxPower(0.3))
                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
                .build();


        row2ToLaunch = follower
                .pathBuilder()
                .addPath(new BezierCurve(postIntake2, afterRow2Control, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

//        launchToRow1 = follower
//                .pathBuilder()
//                .addPath(new BezierLine(launchPose, preIntake1))
//                .setLinearHeadingInterpolation(launchPose.getHeading(), preIntake1.getHeading())
//                .addPath(new BezierLine(preIntake1, postIntake1))
//                .setLinearHeadingInterpolation(preIntake1.getHeading(), postIntake1.getHeading())
//                .addParametricCallback(0, () -> follower.setMaxPower(0.3))
//                .addParametricCallback(1.0, () -> follower.setMaxPower(1.0))
//                .build();

        launchToRow1 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, row1Control, postIntake1))
                .setConstantHeadingInterpolation(postIntake1.getHeading())
                .addParametricCallback(0.45, () -> follower.setMaxPower(0.3))
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
