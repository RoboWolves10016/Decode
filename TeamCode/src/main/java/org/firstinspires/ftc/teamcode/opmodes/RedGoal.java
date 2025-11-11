package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    // Paths
    PathChain startToLaunch;
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
        updateTelemetry();
        RobotState.getInstance().addTelemetry(telemetryM);

        switch (autonState) {
            case 1:
                follower.followPath(startToLaunch);
//                follower.followPath(path);
                launcher.setAuto();
                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    advanceAutonState();
                }
                break;
            case 2:
                follower.breakFollowing();
                kicker.feed();
                break;
        }
        follower.update();
        Tuning.Drawing.drawDebug(follower);
        telemetryM.update(telemetry);
        drivetrain.run();
        intake.run();
        spindexer.run();
        kicker.run();
        launcher.run();
        limelight.run();
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
                .setConstantHeadingInterpolation(Math.toRadians(35.3))
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
