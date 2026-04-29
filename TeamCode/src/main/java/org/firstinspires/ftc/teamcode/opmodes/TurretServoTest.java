package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Configurable
@TeleOp(name="Turret Servo Test", group="Test")
public class TurretServoTest extends OpMode {
    Turret turret;
    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    double setpoint = 0.5;
    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        telemetry.addData("setpoint", setpoint);
    }
}
