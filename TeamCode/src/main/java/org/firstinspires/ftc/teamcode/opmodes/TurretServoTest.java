package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;
import com.seattlesolvers.solverslib.util.MathUtils;

@TeleOp(name="Turret Servo Test", group="Test")
public class TurretServoTest extends OpMode {
    ServoEx servo1;
    ServoEx servo2;
    ServoExGroup servos;

    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    double setpoint = 0.5;
    @Override
    public void init() {
        servo1 = new ServoEx(hardwareMap, "TurretServo1");
        servo2 = new ServoEx(hardwareMap, "TurretServo2");

        servos = new ServoExGroup(servo1, servo2);
        servos.setInverted(false);
        servos.setCachingTolerance(0.0005);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper && !lastRightBumper) setpoint += 0.05;
        if (gamepad1.left_bumper && !lastLeftBumper) setpoint -= 0.05;

        if (gamepad1.right_trigger_pressed) setpoint += 0.01;
        if (gamepad1.left_trigger_pressed) setpoint -= 0.01;
        setpoint = MathUtils.clamp(setpoint, -1.0, 1.0);
        servos.set(setpoint);


        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        telemetry.addData("setpoint", setpoint);
    }
}
