package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.ArrayList;
import java.util.List;

import kotlin.IgnorableReturnValue;

public class CloseAutonPaths {

    // Poses are BLUE by default, mirror for RED
    private Alliance alliance = Alliance.BLUE;

    public Pose startPose;
    private Pose launchPose;

    private Pose row2Control;
    private Pose row2End;
    private Pose row2ReturnControl;

    private Pose gateControl;
    private Pose gateEnd;
    private Pose gateExtraEnd;
    private Pose gateReturnControl;

    private Pose row1End;
    private Pose endPose;

    /** FAR AUTON PATHS **/
    public PathChain startToLaunch;
    public PathChain launchToRow2;
    public PathChain row2ToLaunch;
    public PathChain launchToGate;
    public PathChain gateToLaunch;
    public PathChain launchToRow1;
    public PathChain row1ToLaunch;
    public PathChain launchToEnd;

    public CloseAutonPaths(Alliance alliance, Follower follower) {
        startPose = new Pose(31, 136, Math.toRadians(270));
        launchPose = new Pose(50, 87, Math.toRadians(-115));

        row2Control = new Pose(44, 64);
        row2End = new Pose(15, 60, Math.toRadians(180));
        row2ReturnControl = new Pose(43, 73);

        gateControl = new Pose(34, 59);
        gateEnd = new Pose(14, 62, Math.toRadians(155));
        gateReturnControl = new Pose(41, 66);

        gateExtraEnd = new Pose(13, 58, Math.toRadians(140));

        row1End = new Pose(18, 84, Math.toRadians(180));

        endPose = new Pose(24,62, Math.toRadians(180));

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

            gateControl = gateControl.mirror();
            gateEnd = gateEnd.mirror();
            gateExtraEnd = gateExtraEnd.mirror();
            gateReturnControl = gateReturnControl.mirror();

            row1End = row1End.mirror();
            endPose = endPose.mirror();
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

        launchToGate = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, gateControl, gateEnd))
                .setLinearHeadingInterpolation(launchPose.getHeading(), gateEnd.getHeading())
                .setVelocityConstraint(0)
                .addPath(new BezierLine(gateEnd, gateExtraEnd))
                .setLinearHeadingInterpolation(gateEnd.getHeading(), gateExtraEnd.getHeading())
                .build();

        gateToLaunch = follower.pathBuilder()
                .addPath(new BezierCurve(gateEnd, gateReturnControl, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        launchToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, row1End))
                .setTangentHeadingInterpolation()
                .build();

        row1ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(row1End, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        launchToEnd = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();

    }
}
