package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class FarAutonPathsOld {


    // Poses are BLUE by default, mirror for red
    public static Pose startingPose = new Pose(55.5, 7.44, Math.toRadians(180));
    public static Pose launchPose = new Pose(58, 20, Math.toRadians(210));

    // Poses for intaking from row 3
    public static Pose row3Control = new Pose(45, 34);
    public static Pose row3End = new Pose(17, 32, Math.toRadians(180));

    // Poses for intaking from corner
//    public static Pose cornerIntake1Control = new Pose(8, 45);
//    public static Pose cornerIntake1End = new Pose(10, 11.5, Math.toRadians(270));
//    public static Pose cornerIntake1ReturnControl = new Pose();

    // Poses for intaking from secret tunnel
//    public static Pose cornerIntake2Control = new Pose();
    public static Pose cornerIntake2End = new Pose(11, 14, Math.toRadians(180));
    public static Pose cornerIntake2Mid = new Pose(18, 14, Math.toRadians(180));
    public static Pose cornerIntake2Final = new Pose(14, 8, Math.toRadians(180));

    public static Pose cornerIntake3Control = new Pose(26, 12);
    public static Pose cornerIntake3End = new Pose(13, 32, Math.toRadians(150));


    /** FAR AUTON PATHS **/
    public static PathChain startToLaunch;
    public static PathChain launchToRow3;
    public static PathChain row3ToLaunch;
//    public static PathChain launchToCorner1;
//    public static PathChain corner1ToLaunch;

    public static PathChain launchToCorner2;
    public static PathChain corner2ToLaunch;

    public static PathChain launchToCorner3;
    public static PathChain corner3ToLaunch;

    public static void setAlliance(Alliance alliance) {
        if (alliance == Alliance.RED) {
            startingPose = startingPose.mirror();
            launchPose = launchPose.mirror();
            row3Control = row3Control.mirror();
            row3End = row3End.mirror();
//            cornerIntake1Control = cornerIntake1Control.mirror();
//            cornerIntake1End = cornerIntake1End.mirror();
//            cornerIntake1ReturnControl = cornerIntake1ReturnControl.mirror();
//            cornerIntake2Control = cornerIntake1Control.mirror();
            cornerIntake2End = cornerIntake2End.mirror();
            cornerIntake2Mid = cornerIntake2Mid.mirror();
            cornerIntake2Final = cornerIntake2Final.mirror();
            cornerIntake3Control = cornerIntake3Control.mirror();
            cornerIntake3End = cornerIntake3End.mirror();
//            cornerIntake2ReturnControl = cornerIntake1ReturnControl.mirror();
        }
    }

    public static void createPaths(Follower follower) {
        startToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(
                        startingPose,
                        launchPose
                )).setLinearHeadingInterpolation(startingPose.getHeading(), launchPose.getHeading())
                .build();

        launchToRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                                launchPose,
                                row3Control,
                                row3End)
                ).setTangentHeadingInterpolation()
                .build();

        row3ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(
                        row3End,
                        launchPose)
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.5, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(0.5, 1.0, HeadingInterpolator.constant(startingPose.getHeading()))
                )).build();

//        launchToCorner1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        launchPose,
//                        cornerIntake1Control,
//                        cornerIntake1End
//                )).setTangentHeadingInterpolation()
//                .build();

//        corner1ToLaunch = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        cornerIntake1End,
//                        cornerIntake1ReturnControl,
//                        launchPose
//                )).setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
//                .build();

        launchToCorner2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        launchPose,
//                        cornerIntake2Control,
                        cornerIntake2End
                )).setHeadingInterpolation(HeadingInterpolator.tangent)
                .addParametricCallback(0.3, () -> follower.setMaxPower(0.4))
//                .addPath(new BezierLine(
//                        cornerIntake2End,
//                        cornerIntake2Mid
//                )).setHeadingInterpolation(HeadingInterpolator.constant(cornerIntake2End.getHeading()))
//                .addParametricCallback(0.1, () -> follower.setMaxPower(0.75))
//                .addPath(new BezierLine(
//                        cornerIntake2Mid,
//                        cornerIntake2Final
//                )).setHeadingInterpolation(HeadingInterpolator.tangent)
//                .addParametricCallback(0.2, () -> follower.setMaxPower(0.4))
                .build();

        corner2ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(
                        cornerIntake2Final,
                        launchPose
                )).setHeadingInterpolation(HeadingInterpolator.linear(cornerIntake2End.getHeading(), startingPose.getHeading()))
                .addParametricCallback(0.0, () -> follower.setMaxPower(0.75))
                .build();

        launchToCorner3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        cornerIntake3Control,
                        cornerIntake3End
                )).setHeadingInterpolation(HeadingInterpolator.tangent)
                .build();

        corner3ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(
                        cornerIntake3End,
                        launchPose
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.5, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(0.5, 1.0, HeadingInterpolator.constant(launchPose.getHeading()))
                )).build();

    }
}
