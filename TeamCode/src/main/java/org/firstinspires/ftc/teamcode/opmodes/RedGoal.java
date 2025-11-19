package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.GoalAutonPoses.endPose;
import static org.firstinspires.ftc.teamcode.opmodes.GoalAutonPoses.launchPose;
import static org.firstinspires.ftc.teamcode.opmodes.GoalAutonPoses.postIntake1;
import static org.firstinspires.ftc.teamcode.opmodes.GoalAutonPoses.postIntake2;
import static org.firstinspires.ftc.teamcode.opmodes.GoalAutonPoses.postIntake3;
import static org.firstinspires.ftc.teamcode.opmodes.GoalAutonPoses.row1Control;
import static org.firstinspires.ftc.teamcode.opmodes.GoalAutonPoses.row2Control;
import static org.firstinspires.ftc.teamcode.opmodes.GoalAutonPoses.row3Control;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedropathing.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BallState;
import org.firstinspires.ftc.teamcode.util.PoseUtils;
import org.firstinspires.ftc.teamcode.util.SpindexerSlot;

@Autonomous(name = "Red Goal")
public class RedGoal extends OpMode {
    private final RobotState robotState = RobotState.getInstance();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private final Alliance alliance = Alliance.RED;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    // Paths
    PathChain startToLaunch;
    PathChain launchToRow3;
    PathChain row3ToLaunch;
    PathChain launchToRow2;
    PathChain row2ToLaunch;
    PathChain launchToRow1;
    PathChain row1ToLaunch;
    PathChain launchToEnd;
    private Drive drivetrain;
    private Follower follower;
    private Launcher launcher;
    private Spindexer spindexer;
    private Intake intake;
    private Limelight limelight;
    private Kicker kicker;
    private int autonState = 0;

    @Override
    public void init() {
        drivetrain = new Drive(hardwareMap, new GamepadEx(gamepad1));
        drivetrain.init();
        drivetrain.startAuton();
        follower = drivetrain.getFollower();
        follower.setStartingPose(new Pose(125.135, 121.153, Math.toRadians(35.3)));
        launcher = new Launcher(hardwareMap);
        launcher.init();
        spindexer = new Spindexer(hardwareMap);
        spindexer.init();
        kicker = new Kicker(hardwareMap);
        kicker.init();
        intake = new Intake(hardwareMap);
        intake.init();
        limelight = new Limelight(hardwareMap);
        limelight.init();

        robotState.setAlliance(alliance);
        robotState.setLimelightEnabled(false);
        GoalAutonPoses.setAlliance(alliance);
        createPaths();

    }

    @Override
    public void init_loop() {
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        spindexer.setSlotData(SpindexerSlot.ONE, BallState.GREEN);
        spindexer.setSlotData(SpindexerSlot.TWO, BallState.PURPLE);
        spindexer.setSlotData(SpindexerSlot.THREE, BallState.PURPLE);
        spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
        spindexer.setFeedType(Spindexer.FeedType.PATTERN);
        spindexer.setLaunchMode();
        stateTimer.reset();
    }

    @Override
    public void loop() {
        updateTelemetry();
        RobotState.getInstance().addTelemetry(telemetryM);

        switch (autonState) {
            case 0:
                follower.followPath(startToLaunch);
                advanceAutonState();
                break;
            case 1:
                // Go to launch position
//                follower.followPath(startToLaunch);
                launcher.setAuto();
                if (!follower.isBusy() || stateTimer.seconds() > 2) {
                    advanceAutonState();
                }
                break;
            case 2:
                // Shoot
                intake.stopIntake();
                kicker.feed();
                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    follower.followPath(launchToRow3);
                }
                break;
            case 3:
                // Follow ball intake path and intake
                launcher.setIdle();
                intake.runIntake();
                kicker.resetHistory();
                spindexer.setIntakeMode();

                if (!follower.isBusy() ||  stateTimer.seconds() > 5) {
                    advanceAutonState();
                    follower.followPath(row3ToLaunch);
                    follower.setMaxPower(1.0);
                }
                break;
            case 4:
                if (stateTimer.seconds() > 1) advanceAutonState();
            case 5:
                // Go to launch position
                launcher.setAuto();

                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    advanceAutonState();
                    spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
                    spindexer.setFeedType(Spindexer.FeedType.PATTERN);
                    spindexer.setLaunchMode();
                }
                break;
            case 6:
                // Shoot
                intake.stopIntake();
                kicker.feed();

                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    kicker.stopFeed();
                    launcher.setIdle();
                    follower.followPath(launchToRow2);
                }
                break;
            case 7:
                // Start intaking and drive to row 2
                intake.runIntake();
                spindexer.setIntakeMode();
                kicker.resetHistory();

                if (!follower.isBusy() || stateTimer.seconds() > 7) {
                    advanceAutonState();
                    follower.setMaxPower(1.0);
                    follower.followPath(row2ToLaunch);
                }
                break;
            case 8:
                if (stateTimer.seconds() > 1) advanceAutonState();
            case 9:
                // Drive back to launch position
                launcher.setAuto();

                if (!follower.isBusy() || stateTimer.seconds() > 3) {
//                    advanceAutonState();
                    autonState = 14;
                    stateTimer.reset();
                    spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
                    spindexer.setFeedType(Spindexer.FeedType.PATTERN);
                    spindexer.setLaunchMode();
                }
                break;

            case 10:
                // Shoot
                intake.stopIntake();
                kicker.feed();
                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    kicker.stopFeed();
                    launcher.setIdle();
                    follower.followPath(launchToRow1);
                }
                break;
            case 11:
                // Start intaking and drive to row 1
                intake.runIntake();
                kicker.resetHistory();
                spindexer.setIntakeMode();

                if (!follower.isBusy() || stateTimer.seconds() > 10) {
                    advanceAutonState();
                    follower.setMaxPower(1.0);
                    follower.followPath(row1ToLaunch);
                }
                break;
            case 12:
                if (stateTimer.seconds() > 1) advanceAutonState();
            case 13:
                // Drive back to launch position
                launcher.setAuto();
                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    advanceAutonState();
                    spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
                    spindexer.setFeedType(Spindexer.FeedType.PATTERN);
                    spindexer.setLaunchMode();
                }
                break;
            case 14:
                // Launch
                intake.stopIntake();
                kicker.feed();
                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    kicker.stopFeed();
                    launcher.setIdle();
                    follower.followPath(launchToEnd);
                }
                break;
            case 15:
                // Move off line
                if (!follower.isBusy()) {
                    // Do Nothing
                }
                break;
        }

        Tuning.Drawing.drawDebug(follower);
        drivetrain.run();
        intake.run();
        spindexer.run();
        kicker.run();
        launcher.run();
        limelight.run();
        telemetryM.update(telemetry);
    }

    private void updateTelemetry() {
        telemetryM.addLine("---------PEDRO AUTON---------");
        telemetryM.addData("State Duration", stateTimer.seconds());
        telemetryM.addData("Auton State", autonState);
        telemetryM.addData("Is Busy", follower.isBusy());
        telemetryM.addData("Follower Pose", follower.getPose());
        telemetryM.addData("T-Value", follower.getCurrentTValue());
        telemetryM.addData("Path Number", follower.getCurrentPathNumber());
        telemetryM.update(telemetry);
    }

    private void advanceAutonState() {
        autonState = autonState + 1;
        stateTimer.reset();
    }


    private void createPaths() {
        startToLaunch = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(GoalAutonPoses.startPose, GoalAutonPoses.launchPose)
                )
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

        launchToRow3 = follower
                .pathBuilder()
                .addPath(new BezierCurve(launchPose, row3Control, postIntake3))
//                .setLinearHeadingInterpolation(launchPose.getHeading(), postIntake3.getHeading())
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.2,
                                HeadingInterpolator.linear(launchPose.getHeading(), postIntake3.getHeading())),
                        new HeadingInterpolator.PiecewiseNode(
                                0.2,
                                1.0,
                                HeadingInterpolator.constant(postIntake3.getHeading())
                        )
                ))
                .addParametricCallback(0.2, () -> follower.setMaxPower(0.3))
                .build();

        row3ToLaunch = follower
                .pathBuilder()
                .addPath(new BezierLine(postIntake3, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

        launchToRow2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(launchPose, row2Control, postIntake2))
//                .setLinearHeadingInterpolation(launchPose.getHeading(), postIntake2.getHeading())
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.5,
                                HeadingInterpolator.linear(launchPose.getHeading(), postIntake2.getHeading())),
                        new HeadingInterpolator.PiecewiseNode(
                                0.5,
                                1.0,
                                HeadingInterpolator.constant(postIntake3.getHeading())
                        )
                ))

                .addParametricCallback(0.5, () -> follower.setMaxPower(0.3))
                .build();

        row2ToLaunch = follower
                .pathBuilder()
                .addPath(new BezierLine(postIntake2, launchPose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(alliance.goalPose))
                .build();

        launchToRow1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(launchPose, row1Control, postIntake1))
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.4))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.5,
                                HeadingInterpolator.linear(launchPose.getHeading(), postIntake1.getHeading())),
                        new HeadingInterpolator.PiecewiseNode(
                                0.5,
                                1.0,
                                HeadingInterpolator.constant(postIntake3.getHeading())
                        )
                ))
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

    public void drawOnlyCurrent() {
        try {
            Tuning.Drawing.drawRobot(follower.getPose());
            Tuning.Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }


}
