package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="Full Turret Test")
public class TurretTest extends OpMode {
    private Turret turret;
    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        turret.init();
    }

    @Override
    public void loop() {
        turret.run();
    }
}
