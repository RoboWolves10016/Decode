package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.Drivetrain;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@TeleOp(name="Test")
public class Test extends OpMode {
    Limelight limelight;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        limelight.periodic();
    }
}
