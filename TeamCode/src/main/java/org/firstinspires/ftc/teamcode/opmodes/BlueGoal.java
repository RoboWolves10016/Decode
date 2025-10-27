package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.SpindexerSlot;

@Autonomous(name = "Blue Goal")
public class BlueGoal extends OpMode {
    private final RobotState robotState = RobotState.getInstance();
    private Drive drivetrain;
    private Follower follower;
    private Launcher launcher;
    private Spindexer spindexer;
    private Kicker kicker;
    private PathChain pathChain;
    private ElapsedTime timer = new ElapsedTime();
    private double shootingStartTime = 0;
    private boolean startedLaunching = false;
    private boolean doneLaunching = false;

    @Override
    public void init() {
        drivetrain = new Drive(hardwareMap,new GamepadEx(gamepad1));
        drivetrain.init();
        follower = drivetrain.getFollower();

        launcher = new Launcher(hardwareMap);
        launcher.init();
        spindexer = new Spindexer(hardwareMap);
        spindexer.init();
        kicker = new Kicker(hardwareMap);
        kicker.init();

        robotState.setAlliance(Alliance.BLUE);
        follower.setStartingPose(new Pose(26, 130, Math.toRadians(144)));
        pathChain = createPathChain(follower);
    }

    @Override
    public void start() {
        launcher.setAuto();
        spindexer.setLaunchMode();
        timer.reset();
    }

    @Override
    public void loop() {
        follower.followPath(pathChain);
        follower.update();

        if (!follower.isBusy() && !doneLaunching) {
            kicker.feed();
            if (!startedLaunching) {
                startedLaunching = true;
                shootingStartTime = timer.seconds();
            }
            if (timer.seconds() - shootingStartTime > 0.25) spindexer.stepCounterClockwise();
            spindexer.stepCounterClockwise();
            if (startedLaunching && robotState.getCurrentSlot() == SpindexerSlot.ONE) {
                doneLaunching = true;
            }
        } else {
            launcher.setIdle();
            kicker.stopFeed();
        }
    }

    public PathChain createPathChain(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(26, 130), new Pose(53, 115))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(150))
                .build();
    }


}
