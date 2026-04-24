package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.RobotState;

import lombok.Setter;

public class Hood extends Subsystem{
    // Required subsystem components
    private final TelemetryManager telemetry;
    private final HardwareMap hardwareMap;
    private final RobotState robotState = RobotState.getInstance();

    private ServoEx servo;

    @Setter
    private double targetDeg = BOTTOM_HOOD_ANGLE;
    private double targetPos = BOTTOM_HOOD_POS;

    public static boolean useManualOverride = false;
    public static double manualOverrideDeg = 22d;

    public Hood(HardwareMap hwMap) {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        hardwareMap = hwMap;
    }

    @Override
    void init() {
        servo = new ServoEx(hardwareMap, "HoodServo");
        servo.setCachingTolerance(
                BOTTOM_HOOD_POS - angleToPos(BOTTOM_HOOD_ANGLE + HOOD_CACHING_TOL_DEG)
        );
    }

    @Override
    void run() {
        targetPos = angleToPos(targetDeg);

        servo.set(MathUtils.clamp(targetPos, TOP_HOOD_POS, BOTTOM_HOOD_POS));
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------HOOD--------------");
        telemetry.addData("Target Angle (Deg)", targetDeg);
        telemetry.addData("Target Pos", targetPos);
    }

    @Override
    void stop() {

    }

    private double angleToPos(double hoodDegrees) {
        if (Double.isNaN(hoodDegrees)) {
            return Double.NaN;
        }
        if (hoodDegrees < BOTTOM_HOOD_ANGLE) return BOTTOM_HOOD_POS;
        if (hoodDegrees > TOP_HOOD_ANGLE) return TOP_HOOD_POS;
        return BOTTOM_HOOD_POS
                + (hoodDegrees - TOP_HOOD_ANGLE) * (TOP_HOOD_POS - BOTTOM_HOOD_POS)
                                                / (TOP_HOOD_ANGLE - BOTTOM_HOOD_ANGLE);

    }
}
