package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

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
import org.firstinspires.ftc.teamcode.util.SpindexerSlot;

@Autonomous(name = "Red Goal")
public class RedGoal extends OpMode {
    private final RobotState robotState = RobotState.getInstance();
    private final ElapsedTime stateTimer = new ElapsedTime();
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    // Paths
    PathChain startToLaunch;
    PathChain launchToRow3;
    PathChain row3ToLaunch;
    Path path;
    private Drive drivetrain;
    private Follower follower;
    private Launcher launcher;
    private Spindexer spindexer;
    private Intake intake;
    private Limelight limelight;
    private Kicker kicker;
    private int autonState = 1;

    @Override
    public void init() {
        drivetrain = new Drive(hardwareMap, new GamepadEx(gamepad1));
        drivetrain.init();
        drivetrain.startAuton();
        follower = drivetrain.getFollower();
//        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(125.135, 121.153, Math.toRadians(35.3)));
//        follower.setStartingPose(new Pose(72, 72));
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

        robotState.setAlliance(Alliance.RED);
        robotState.setLimelightEnabled(false);
        robotState.setLimelightEnabled(false);
    }

    @Override
    public void init_loop() {
        follower.update();
        drawOnlyCurrent();

    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        createPaths();
//        path = new Path(new BezierLine(new Pose(72, 72), new Pose(112, 72)));

        spindexer.setSlotData(SpindexerSlot.ONE, BallState.GREEN);
        spindexer.setSlotData(SpindexerSlot.TWO, BallState.PURPLE);
        spindexer.setSlotData(SpindexerSlot.THREE, BallState.PURPLE);
        spindexer.setLaunchMode();
        stateTimer.reset();
    }

    @Override
    public void loop() {
//        updateTelemetry();
        RobotState.getInstance().addTelemetry(telemetryM);

        switch (autonState) {
            case 1:
                // Go to launch position
                follower.followPath(startToLaunch);
                launcher.setAuto();
                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    advanceAutonState();
                    follower.pausePathFollowing();
                }
                break;
            case 2:
                // Shoot
                follower.breakFollowing();
                kicker.feed();
                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    follower.resumePathFollowing();
                }
                break;
            case 3:
                // Go to ball intake position and intake
                follower.followPath(launchToRow3);
                launcher.setIdle();
                spindexer.setIntakeMode();
                intake.runIntake();
                kicker.resetHistory();

                if (!follower.isBusy()) {
                    advanceAutonState();
                }
            case 4:
                // Go to shoot position
                follower.followPath(row3ToLaunch);
                spindexer.setLaunchMode();
                launcher.setAuto();

                if (!follower.isBusy()) {
                    advanceAutonState();
                }
            case 5:
                // Shoot
                follower.pausePathFollowing();
                intake.stopIntake();
                kicker.feed();

                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    follower.resumePathFollowing();
                }
                break;
            case 6:
                // Drive to row 2
                launcher.setIdle();
                intake.runIntake();
                spindexer.setIntakeMode();
                kicker.resetHistory();
                break;
        }

//        follower.update();
//        Tuning.Drawing.drawDebug(follower);
        drivetrain.run();
        intake.run();
        spindexer.run();
        kicker.run();
        launcher.run();
        limelight.run();
        telemetryM.update(telemetry);
    }

    private void updateTelemetry() {
        telemetryM.addLine("---------AUTON---------");
        telemetryM.addData("State Duration", stateTimer.seconds());
        telemetryM.addData("Auton State", autonState);
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
                        new BezierLine(RedGoalPaths.startPose, RedGoalPaths.launchPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(35.3), Math.toRadians(50))
                .build();

        launchToRow3 = follower
                .pathBuilder()
                .addPath(new BezierLine(RedGoalPaths.launchPose, RedGoalPaths.preIntake3))
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                .addPath(new BezierLine(RedGoalPaths.preIntake3, RedGoalPaths.postIntake3))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        row3ToLaunch = follower
                .pathBuilder()
                .addPath(new BezierLine(RedGoalPaths.postIntake3, RedGoalPaths.launchPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                .build();

//        path = new Path(new BezierLine(RedGoalPaths.startPose, RedGoalPaths.launchPose));
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
