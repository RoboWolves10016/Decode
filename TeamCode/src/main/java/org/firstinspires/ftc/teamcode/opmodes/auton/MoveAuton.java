package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name="Move")
public class MoveAuton extends OpMode {
    RobotState robotState;
    Follower follower;
    ElapsedTime timer;

    private static final Pose redStartingPose = new Pose(102,7.44, Math.toRadians(0));
    private static final Pose blueStartingPose = new Pose(42, 7.44, Math.toRadians(180));

    @Override
    public void init() {
        robotState = RobotState.getInstance();

        follower = Constants.createFollower(hardwareMap);
        timer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b) robotState.setAlliance(Alliance.RED);
        if (gamepad1.x) robotState.setAlliance(Alliance.BLUE);

        telemetry.addData("Alliance", robotState.getAlliance());
    }

    @Override
    public void start() {
        follower.setPose(robotState.getAlliance() == Alliance.RED ? redStartingPose : blueStartingPose);
        follower.startTeleopDrive(false);
        timer.reset();
    }

    @Override
    public void loop() {
        if (timer.seconds() < 1) follower.setTeleOpDrive(0.3, 0, 0, true);
        if (timer.seconds() >= 1) follower.setTeleOpDrive(0, 0, 0, false);
        follower.update();
        robotState.setPose(follower.getPose());
    }
}
