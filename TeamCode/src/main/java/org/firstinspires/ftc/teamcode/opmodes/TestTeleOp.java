package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@TeleOp(name="TeleOp", group="Competition")
public class TestTeleOp extends OpMode {

    private Follower follower;
    private Limelight limelight;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        limelight = new Limelight(hardwareMap, telemetry);
        limelight.init();

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        limelight.periodic();
//        follower.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                false);
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
