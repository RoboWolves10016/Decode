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

    /** FAR AUTON PATHS **/
    public PathChain startToLaunch;
    public PathChain launchToRow2;
    public PathChain row2ToLaunch;
    public PathChain launchToGate1;
    public PathChain gate1ToGate2;
    public PathChain gate2ToLaunch;
    public PathChain launchToRow1;
    public PathChain row1ToLaunch;
    public PathChain launchToEnd;

    public CloseAutonPaths(Alliance alliance, Follower follower) {
        startPose = new Pose(31, 136, Math.toRadians(270));
        launchPose = new Pose(50, 87, Math.toRadians(-115));

        row2Control = new Pose(44, 68);
        row2End = new Pose(18, 64, Math.toRadians(180));
        row2ReturnControl = new Pose(43, 73);

        gateControl1 = new Pose(40, 67);
        gateEnd1 = new Pose(16, 66, Math.toRadians(150));

//        gateControl2 = new Pose(41, 66);
        gateEnd2 = new Pose(12, 53, Math.toRadians(120));

        gateReturnControl = new Pose(45, 60);

        row1End = new Pose(22, 84, Math.toRadians(180));

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

            gateControl1 = gateControl1.mirror();
            gateEnd1 = gateEnd1.mirror();
            gateEnd2 = gateEnd2.mirror();
//            gateControl2 = gateControl2.mirror();

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

        launchToGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, gateControl1, gateEnd1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), gateEnd1.getHeading())
                .setVelocityConstraint(20)
                .setBrakingStart(0.5)
                .build();

        gate1ToGate2 = follower.pathBuilder()
                .addPath(new BezierLine(gateEnd1, gateEnd2))
                .setConstantHeadingInterpolation(gateEnd2.getHeading())
                .build();


        gate2ToLaunch = follower.pathBuilder()
                .addPath(new BezierCurve(gateEnd2, gateReturnControl, launchPose))
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
