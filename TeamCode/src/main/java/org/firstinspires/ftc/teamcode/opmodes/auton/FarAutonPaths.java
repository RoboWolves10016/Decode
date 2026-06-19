package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class FarAutonPaths {

    // Poses are BLUE by default, mirror for RED
    private Alliance alliance = Alliance.BLUE;

    public Pose startPose;
    public Pose launchPose;

    private Pose row3Control;
    private Pose row3End;;

    private Pose row2Control;
    private Pose row2End;

    private Pose cornerEnd1;
    private Pose cornerRetreat;
    private Pose cornerEnd2;

    private Pose cornerWallControl;
    private Pose wallOnlyControl;
    private Pose wallEnd;

    private Pose endPose;

    public PathChain startToLaunch;
    public PathChain launchToRow3;
    public PathChain row3ToLaunch;
    public PathChain launchToRow2;
    public PathChain row2ToLaunch;
    public PathChain launchToPureCorner1;
    public PathChain corner1ToCorner2;
    public PathChain pureCornerToLaunch;
    public PathChain launchToCornerAndWall;
    public PathChain launchToWall;
    public PathChain wallToLaunch;

    public PathChain launchToEnd;


    public FarAutonPaths(Alliance alliance, Follower follower) {
        startPose = new Pose(55, 7.75, Math.toRadians(180));
        launchPose = new Pose(54, 17, Math.toRadians(180));

        row3Control = new Pose(50, 28);
        row3End = new Pose(18, 30, Math.toRadians(180));

        row2Control = new Pose(61, 61);
        row2End = new Pose(20, 60, Math.toRadians(180));

        cornerEnd1 = new Pose(20, 13, Math.toRadians(180));
        cornerRetreat = new Pose(30, 10, Math.toRadians(180));
        cornerEnd2 = new Pose(20, 7, Math.toRadians(180));

        cornerWallControl = new Pose(10, 17);
        wallOnlyControl = new Pose(16, 23);

        wallEnd = new Pose(10, 41); // Heading not used

        endPose = new Pose(43, 20, Math.toRadians(180));

        setAlliance(alliance);
        createPaths(follower);
    }

    private void setAlliance(Alliance alliance) {
        if (alliance != this.alliance) {
            this.alliance = alliance;
            startPose = startPose.mirror();
            launchPose = launchPose.mirror();

            row3Control = row3Control.mirror();
            row3End = row3End.mirror();

            row2Control = row2Control.mirror();
            row2End = row2End.mirror();

            cornerEnd1 = cornerEnd1.mirror();
            cornerRetreat = cornerRetreat.mirror();
            cornerEnd2 = cornerEnd2.mirror();

            wallEnd = wallEnd.mirror();

            cornerWallControl = cornerWallControl.mirror();
            wallOnlyControl = wallOnlyControl.mirror();

            endPose = endPose.mirror();
        }
    }

    private void createPaths(Follower follower) {
        startToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();

        launchToRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, row3Control, row3End))
//                .addParametricCallback(0.1, () -> follower.setMaxPower(0.5))
                .setTangentHeadingInterpolation()
                .build();

        row3ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(row3End, launchPose))
                .addParametricCallback(0.0, () -> follower.setMaxPower(1.0))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.6, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(0.6, 1.0, HeadingInterpolator.linear(row3End.getHeading(), launchPose.getHeading()))
                )).build();

        launchToRow2 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, row2Control, row2End))
                .setLinearHeadingInterpolation(launchPose.getHeading(), row2End.getHeading())
                .build();

        row2ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(row2End, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.6, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(0.6, 1.0, HeadingInterpolator.linear(row2End.getHeading(), launchPose.getHeading()))
                )).build();

        launchToPureCorner1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, cornerEnd1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), cornerEnd1.getHeading())
                .build();

        corner1ToCorner2 = follower.pathBuilder()
                .addPath(new BezierLine(cornerEnd1, cornerEnd2))
                .setLinearHeadingInterpolation(cornerEnd1.getHeading(), cornerEnd2.getHeading())
                .build();


        pureCornerToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(cornerEnd2, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(0.7, 1.0, HeadingInterpolator.constant(launchPose.getHeading()))
                )).build();

        launchToCornerAndWall = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, cornerEnd1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), cornerEnd1.getHeading())
                .addPath(new BezierLine(cornerEnd1, cornerRetreat))
                .setLinearHeadingInterpolation(cornerEnd1.getHeading(), cornerRetreat.getHeading())
                .addPath(new BezierCurve(cornerRetreat, cornerWallControl, wallEnd))
                .setHeadingInterpolation(HeadingInterpolator.tangent)
                .build();

        launchToWall = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, wallOnlyControl, wallEnd))
                .setTangentHeadingInterpolation()
                .build();

        wallToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(wallEnd, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(0.7, 1.0, HeadingInterpolator.constant(launchPose.getHeading()))
                )).build();

        launchToEnd = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, endPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), endPose.getHeading())
                .build();
    }
}
