package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Motor Wiring Test")
public class MotorTest extends OpMode {

    private DcMotorEx wheel1;
    private DcMotorEx wheel2;
    private DcMotorEx wheel3;
    private DcMotorEx wheel4;

    @Override
    public void init() {
        wheel1 = hardwareMap.get(DcMotorEx.class, "Wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "Wheel2");
        wheel3 = hardwareMap.get(DcMotorEx.class, "Wheel3");
        wheel4 = hardwareMap.get(DcMotorEx.class, "Wheel4");
        telemetry.addData("D-Pad Up", "Wheel1");
        telemetry.addData("D-Pad Right", "Wheel2");
        telemetry.addData("D-Pad Down", "Wheel3");
        telemetry.addData("D-Pad Left", "Wheel4");
        telemetry.update();

    }

    @Override
    public void loop() {
        wheel1.setPower(gamepad1.dpad_up ? 0.3 : 0);
        wheel2.setPower(gamepad1.dpad_right ? 0.3 : 0);
        wheel3.setPower(gamepad1.dpad_down ? 0.3 : 0);
        wheel4.setPower(gamepad1.dpad_left ? 0.3 : 0);
    }
}
