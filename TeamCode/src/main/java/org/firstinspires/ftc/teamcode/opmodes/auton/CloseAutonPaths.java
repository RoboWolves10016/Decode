package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class  CloseAutonPaths {

    // Poses are BLUE by default, mirror for RED
    private Alliance alliance = Alliance.BLUE;

    public Pose startPose;
    private Pose launchPose;

    private Pose row2Control;
    private Pose row2End;
    private Pose row2ReturnControl;

    private Pose gateControl1;
    private Pose gateEnd1;
    private Pose gateEnd2;
//    private Pose gateControl2;

    private Pose gateReturnControl;

    private Pose row1End;
    private Pose endPose;

//    private Pose fullGateControl1;
//    private Pose fullGateControl2;
//    private Pose fullGateControl3;

    private Pose row3Control1;
    private Pose row3Control2;
    private Pose row3End;

    /** FAR AUTON PATHS **/
    public PathChain startToLaunch;
    public PathChain launchToRow2;
    public PathChain row2ToLaunch;
    public PathChain launchToGate1;
    public PathChain gate1ToGate2;

    public PathChain launchToGateFull;
    public PathChain gate2ToLaunch;
    public PathChain launchToRow1;
    public PathChain row1ToLaunch;
    public PathChain launchToRow3;
    public PathChain row3ToLaunch;
    public PathChain launchToEnd;

    public CloseAutonPaths(Alliance alliance, Follower follower) {
        startPose = new Pose(31, 136, Math.toRadians(270));
        launchPose = new Pose(50, 87, Math.toRadians(-115));

        row2Control = new Pose(44, 66);
        row2End = new Pose(16, 64, Math.toRadians(180));
        row2ReturnControl = new Pose(43, 73);

        gateControl1 = new Pose(41, 72);
        gateEnd1 = new Pose(17, 66, Math.toRadians(160));

        gateEnd2 = new Pose(12, 53, Math.toRadians(120));

        gateReturnControl = new Pose(45, 60);

        row1End = new Pose(20, 84, Math.toRadians(180));

        endPose = new Pose(24,62, Math.toRadians(180));

//        fullGateControl1 = new Pose(37, 54);
//        fullGateControl2 = new Pose(13, 97);
//        fullGateControl3 = new Pose(16, 59);

        row3Control1 = new Pose(52, 44);
        row3Control2 = new Pose(43, 38);
        row3End = new Pose(15, 40, Math.toRadians(180));

        setAlliance(alliance);
        createPaths(follower);
    }
    private void setAlliance(Alliance alliance) {
        if (alliance != this.alliance) {
            this.alliance = alliance;
            startPose = startPose.mirror();
            launchPose = launchPose.mirror();

            row2Control = row2Control.mirror();
            row2End = row2End.mirror();
            row2ReturnControl = row2ReturnControl.mirror();

            gateControl1 = gateControl1.mirror();
            gateEnd1 = gateEnd1.mirror();
            gateEnd2 = gateEnd2.mirror();
//            gateControl2 = gateControl2.mirror();

            gateReturnControl = gateReturnControl.mirror();

            row1End = row1End.mirror();
            endPose = endPose.mirror();

//            fullGateControl1 = fullGateControl1.mirror();
//            fullGateControl2 = fullGateControl2.mirror();
//            fullGateControl3 = fullGateControl3.mirror();

            row3Control1 = row3Control1.mirror();
            row3Control2 = row3Control2.mirror();
            row3End = row3End.mirror();
        }
    }

    private void createPaths(Follower follower) {
        startToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        launchPose
                )).setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();

        launchToRow2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        row2Control,
                        row2End)
                ).setLinearHeadingInterpolation(launchPose.getHeading(), row2End.getHeading())
                .build();

        row2ToLaunch = follower.pathBuilder()
                .addPath(new BezierCurve(row2End, row2ReturnControl, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        launchToGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, gateControl1, gateEnd1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), gateEnd1.getHeading())
                .setTimeoutConstraint(0)
                .setTValueConstraint(0.9)
                .build();

        gate1ToGate2 = follower.pathBuilder()
                .addPath(new BezierLine(gateEnd1, gateEnd2))
                .setConstantHeadingInterpolation(gateEnd2.getHeading())
                .build();


        gate2ToLaunch = follower.pathBuilder()
                .addPath(new BezierCurve(gateEnd2, gateReturnControl, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.3, HeadingInterpolator.constant(gateEnd2.getHeading())),
                        new HeadingInterpolator.PiecewiseNode(0.3, 1.0, HeadingInterpolator.tangent.reverse())
                )).build();

//        launchToGateFull = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        launchPose,
//                        fullGateControl1,
//                        fullGateControl2,
//                        fullGateControl3,
//                        gateEnd2
//                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
//                        new HeadingInterpolator.PiecewiseNode(
//                                0.0,
//                                0.5,
//                                HeadingInterpolator.tangent),
//                        new HeadingInterpolator.PiecewiseNode(
//                                0.5,
//                                1.0,
//                                HeadingInterpolator.linear(alliance == Alliance.RED ? 0 : Math.PI, gateEnd2.getHeading()))
//                )).build();

        launchToGateFull = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, gateControl1, gateEnd1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), gateEnd1.getHeading())
                .addParametricCallback(0.35, () -> follower.setMaxPower(0.5))
                .addPath(new BezierLine(gateEnd1, gateEnd2))
                .addParametricCallback(0.0, () -> follower.setMaxPower(1.0))
                .setConstantHeadingInterpolation(gateEnd2.getHeading())
                .build();


        launchToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, row1End))
                .setTangentHeadingInterpolation()
                .build();

        row1ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(row1End, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        launchToRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, row3Control1, row3Control2, row3End))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.5,
                                HeadingInterpolator.linear(launchPose.getHeading(), row3End.getHeading())),
                        new HeadingInterpolator.PiecewiseNode(
                                0.5,
                                1.0,
                                HeadingInterpolator.constant(row3End.getHeading()))
                )).build();

        row3ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(row3End, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        launchToEnd = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();

    }
}
