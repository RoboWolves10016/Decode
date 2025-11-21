package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.ColorInt;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.util.BallState;

public class ColorSensors extends Subsystem {

    private final HardwareMap hwMap;
    private final TelemetryManager telemetry;
    private final ElapsedTime timer = new ElapsedTime();
    private ColorRangeSensor left;
    private ColorRangeSensor right;
    private boolean isValid = false;

    private BallState leftState = BallState.EMPTY;
    private BallState rightState = BallState.EMPTY;

    private float lRed = 0;
    private float lGreen = 0;
    private float lBlue = 0;
//    private int lAlph = 0;
    private double lDist = 0;
    private float rRed = 0;
    private float rGreen = 0;
    private float rBlue = 0;
//    private int rAlph = 0;
    private double rDist = 0;
    private BallState overallState = BallState.EMPTY;

    public ColorSensors(HardwareMap hwMap) {
        this.hwMap = hwMap;
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init() {
        left = hwMap.get(ColorRangeSensor.class, "LeftColor");
        right = hwMap.get(ColorRangeSensor.class, "RightColor");
    }

    @Override
    public void run() {
        NormalizedRGBA leftColors = left.getNormalizedColors();

        lRed = 256 * leftColors.red;
        lGreen = 256 * leftColors.green;
        lBlue = 256 * leftColors.blue;
//        lAlph = left.alpha();
        lDist = left.getDistance(DistanceUnit.MM);

        NormalizedRGBA rightColors = right.getNormalizedColors();
        rRed = 256 * rightColors.red;
        rGreen = 256 * rightColors.green;
        rBlue = 256 * rightColors.blue;
//        rAlph = right.alpha();
        rDist = right.getDistance(DistanceUnit.MM);

        if (Double.isNaN(rDist) || rDist > 200) {
            rightState = BallState.EMPTY;
        } else if (rGreen > rRed && rGreen > rBlue) {
            rightState = BallState.GREEN;
        } else {
            rightState = BallState.PURPLE;
        }

        if (Double.isNaN(lDist) || lDist > 200) {
            leftState = BallState.EMPTY;
        } else if (lGreen > lRed && lGreen > lBlue) {
            leftState = BallState.GREEN;
        } else {
            leftState = BallState.PURPLE;
        }

        BallState newState = BallState.EMPTY;
        if (leftState != BallState.EMPTY && rightState != BallState.EMPTY) {
            if (leftState == rightState) {
                newState = rightState;
            }
        }
        if (newState != overallState) {
            timer.reset();
            overallState = newState;
        }
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------COLOR SENSORS--------------");
        telemetry.addData("Left (r,g,b,a):", lRed + ", " + lGreen + ", " + lBlue);// + ", " + lAlph);
        telemetry.addData("Left Distance", lDist);
        telemetry.addData("Right (r,g,b,a):", rRed + ", " + rGreen + ", " + rBlue);// + ", " + rAlph);
        telemetry.addData("Right Distance", rDist);
        telemetry.addData("LeftState", leftState);
        telemetry.addData("RightState", rightState);
        telemetry.addData("OverallState", overallState);
        telemetry.addData("CurrentStateDuration", timer.seconds());
    }

    @Override
    public void stop() {
    }

    public BallState getCurrentBallState() {
        return overallState;
    }

    public double getCurrentStateDuration() {
        return timer.seconds();
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
