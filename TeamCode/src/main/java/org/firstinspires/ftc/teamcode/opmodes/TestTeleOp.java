package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="TeleOp", group="Competition")
public class TestTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
//        robot = new Robot(hardwareMap);
    }

    @Override
    public void start() {
        robot.follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        robot.follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false);
    }

    @Override
    public void stop() {

    }
}
