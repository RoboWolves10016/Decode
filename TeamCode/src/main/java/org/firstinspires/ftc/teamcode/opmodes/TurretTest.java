package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="Full Turret Test")
public class TurretTest extends OpMode {
    private final TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

    private Turret turret;
    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        turret.init();
    }

    @Override
    public void loop() {
        turret.run();
        telemetryManager.update(telemetry);
    }

    @Override
    public void stop() {

    }
}
