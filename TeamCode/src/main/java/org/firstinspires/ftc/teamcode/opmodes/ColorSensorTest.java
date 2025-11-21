package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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
        @ColorInt int left_argb = left.argb();
        telemetryM.addLine("--------LEFT SENSOR--------");
        telemetryM.addData("Individual RGB", left.red() + ", " + left.green() + ", " + left.blue());
        telemetryM.addData("Red", Color.red(left_argb));
        telemetryM.addData("Blue", Color.blue(left_argb));
        telemetryM.addData("Green", Color.green(left_argb));
        telemetryM.addData("Distance MM", left.getDistance(DistanceUnit.MM));

        NormalizedRGBA right_argb = right.getNormalizedColors();
        telemetryM.addLine("--------RIGHT SENSOR--------");
        telemetryM.addData("Individual RGB", right.red() + ", " + right.green() + ", " + right.blue());
        telemetryM.addData("Red", 256 * right_argb.red);
        telemetryM.addData("Blue", 256 * right_argb.blue);
        telemetryM.addData("Green", 256 * right_argb.green);
        telemetryM.addData("Distance MM", right.getDistance(DistanceUnit.MM));

        telemetryM.update(telemetry);
    }

    private int getRed(@ColorInt int color) {
        return (color >> 16) & 0xFF;
    }

    private int getGreen(@ColorInt int color) {
        return (color >> 8) & 0xFF;
    }

    private int getBlue(@ColorInt int color) {
        return color & 0xFF;
    }

}
