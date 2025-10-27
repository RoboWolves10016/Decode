package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Test")
public class ColorSensorTest extends OpMode {
    private ColorRangeSensor left;
    private ColorRangeSensor right;
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        left = hardwareMap.get(ColorRangeSensor.class, "LeftColor");
        right = hardwareMap.get(ColorRangeSensor.class, "RightColor");
    }

    @Override
    public void loop() {
        telemetryM.addLine("--------LEFT SENSOR--------");
        telemetryM.addData("Red", left.red());
        telemetryM.addData("Blue", left.blue());
        telemetryM.addData("Green", left.green());
        telemetryM.addData("Alpha", left.alpha());
        telemetryM.addData("Distance MM", left.getDistance(DistanceUnit.MM));

        telemetryM.addLine("--------RIGHT SENSOR--------");
        telemetryM.addData("Red", right.red());
        telemetryM.addData("Blue", right.blue());
        telemetryM.addData("Green", right.green());
        telemetryM.addData("Alpha", right.alpha());
        telemetryM.addData("Distance MM", right.getDistance(DistanceUnit.MM));

        telemetryM.update(telemetry);
    }
}
