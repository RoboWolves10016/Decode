package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.LauncherConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.util.Interpolation;

import lombok.Setter;

@Configurable
public class Hood extends Subsystem{
    // Required subsystem components
    private final TelemetryManager telemetry;
    private final HardwareMap hardwareMap;
    private final RobotState robotState = RobotState.getInstance();

    private ServoEx servo;
    private AbsoluteAnalogEncoder encoder;

    private enum HoodState {
        IDLE,
        TRACKING,
        PRESET
    }
    private HoodState state = HoodState.IDLE;

    @Setter
    private double targetDeg = (BOTTOM_HOOD_ANGLE + TOP_HOOD_ANGLE) / 2;
    private double targetPos = (BOTTOM_HOOD_POS + TOP_HOOD_POS) / 2;

    private double hoodPosition = 0;

    public static boolean useManualOverride = false;
    public static double manualOverrideDeg = 22d;

//    @Setter
//    private boolean tracking = false;

    public Hood(HardwareMap hwMap) {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        hardwareMap = hwMap;
    }

    @Override
    public void init() {
        servo = new ServoEx(hardwareMap, "HoodServo");
        encoder = new AbsoluteAnalogEncoder(hardwareMap, "HoodAnalog", 3.3, AngleUnit.DEGREES);
//        servo.setCachingTolerance(
//                BOTTOM_HOOD_POS - angleToPos(BOTTOM_HOOD_ANGLE + HOOD_CACHING_TOL_DEG)
//        );
    }

    @Override
    public void run() {
        hoodPosition = (encoder.getCurrentPosition() / 12) + 19.5;
        double distanceToGoal;
        if (Tuning.SHOOT_WHILE_MOVING) distanceToGoal = robotState.getFutureVectorToGoal().getMagnitude();
        else distanceToGoal = robotState.getVectorToGoal().getMagnitude();

        switch (state) {
            case IDLE:
                targetDeg = (TOP_HOOD_ANGLE + BOTTOM_HOOD_ANGLE) / 2;
                break;
            case TRACKING:
                targetDeg = distanceToHoodAngle(distanceToGoal);
                break;
            case PRESET:
                targetDeg = TOP_HOOD_ANGLE;
                break;
        }

        if (useManualOverride) targetDeg = manualOverrideDeg;

        targetPos = angleToPos(targetDeg);

        if (Math.abs(targetDeg - hoodPosition) > 3 + ((TOP_HOOD_ANGLE - BOTTOM_HOOD_ANGLE) / 2))
            targetPos = (TOP_HOOD_POS + BOTTOM_HOOD_POS) / 2;

        servo.set(targetPos);
        updateTelemetry();
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------HOOD--------------");
        telemetry.addData("State", state.toString());
        telemetry.addData("Target Angle (Deg)", targetDeg);
        telemetry.addData("Target Pos", targetPos);
        telemetry.addData("Encoder Degrees", hoodPosition);
        telemetry.addData("Manual Angle", manualOverrideDeg);
//        telemetry.addData("Test", );
    }

    @Override
    public void stop() {

    }

    public void setIdle() {
        state = HoodState.IDLE;
    }

    public void setTracking() {
        state = HoodState.TRACKING;
    }

    public void setPreset() {
        state = HoodState.PRESET;
    }

    private double angleToPos(double hoodDegrees) {
        if (Double.isNaN(hoodDegrees)) {
            return Double.NaN;
        }
        if (hoodDegrees < BOTTOM_HOOD_ANGLE) return BOTTOM_HOOD_POS;
        if (hoodDegrees > TOP_HOOD_ANGLE) return TOP_HOOD_POS;
        return BOTTOM_HOOD_POS
                + (hoodDegrees - BOTTOM_HOOD_ANGLE) * (TOP_HOOD_POS - BOTTOM_HOOD_POS)
                                                / (TOP_HOOD_ANGLE - BOTTOM_HOOD_ANGLE);

    }

    private double distanceToHoodAngle(double distanceFromGoal) {
        return Interpolation.interpolate(LauncherConstants.SHOT_DISTANCES, SHOT_ANGLES, distanceFromGoal);
    }
}
