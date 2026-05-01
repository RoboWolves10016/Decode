package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.LauncherConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.util.Alliance;

import lombok.Setter;

@Configurable
public class Turret extends Subsystem {

    private static final double GEARBOX_RATIO = (40d / 20d) * (48d / 20d);
    private static final double TURRET_RATIO = GEARBOX_RATIO * (22d / 101d);

    // Required subsystem components
    private final TelemetryManager telemetry;
    private final HardwareMap hwMap;
    private final RobotState robotState = RobotState.getInstance();

    public static boolean useManualOverride = false;
    public static double manualOverrideDegrees = 0d;

    private double angleToGoal = 0;
    private double targetDeg = 0;
    private double targetPos = 0.5;
    private double encoderAngle = 0;

    private ServoExGroup turretServos;
    private ServoEx servo1;
    private ServoEx servo2;
    private AbsoluteAnalogEncoder encoder;

//    @Setter
//    private boolean tracking = false;

    private boolean presetShot = false;

    private enum TurretState {
        AIMING,
        IDLE,
        PRESET
    }

    private TurretState currentState = TurretState.IDLE;

    public Turret(HardwareMap hwMap) {
        this.hwMap = hwMap;
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init() {
        servo1 = new ServoEx(hwMap, "Turret1");
        servo2 = new ServoEx(hwMap, "Turret2");

        encoder = new AbsoluteAnalogEncoder(hwMap, "TurretAnalog", 3.3, AngleUnit.DEGREES);

        turretServos = new ServoExGroup(servo1, servo2);
        turretServos.setCachingTolerance(0.0001);
        servo1.getServo().getController().pwmEnable();
        servo2.getServo().getController().pwmEnable();
    }

    @Override
    public void run() {
        encoderAngle = (-encoder.getCurrentPosition() * TURRET_RATIO) + 194;
        if (Tuning.SHOOT_WHILE_MOVING)
            angleToGoal = robotState.getFutureVectorToGoal().getTheta() - robotState.getPose().getHeading() + Math.PI;
        else
            angleToGoal = robotState.getVectorToGoal().getTheta() - robotState.getPose().getHeading() + Math.PI;

        switch (currentState) {
            case IDLE:
                targetDeg = 0;
                break;
            case AIMING:
                targetDeg = Math.toDegrees(MathUtils.normalizeAngle(angleToGoal, false, AngleUnit.RADIANS));
                break;
            case PRESET:
                Pose presetPose = robotState.getPose().getX() > 72 ? RED_SIDE_PRESET_POSE : BLUE_SIDE_PRESET_POSE;
                presetPose = presetPose.setHeading(robotState.getPose().getHeading());
                targetDeg = robotState.getAlliance().goalPose.minus(presetPose).getAsVector().getTheta() + Math.PI;
                targetDeg = Math.toDegrees(targetDeg);
                break;
        }

        if (useManualOverride) {
            targetDeg = manualOverrideDegrees;
        }

//        targetDeg -= TURRET_ROT_FF * robotState.getAngularVelocity();

        targetDeg = MathUtils.clamp(targetDeg, MIN_TURRET_ANGLE_LIMIT, MAX_TURRET_ANGLE_LIMIT);

        // Prevent limit wraparounds
        if (targetDeg > 0 && encoderAngle < 0 && Math.abs(targetDeg - encoderAngle) > 200) {
            targetDeg = -20;
        }

        if (targetDeg < 0 && encoderAngle > 0 && Math.abs(targetDeg - encoderAngle) > 200) {
            targetDeg = 20;
        }

        targetPos = angleToPos(targetDeg);
//        targetPos = 0.5;

        servo1.set(targetPos);
        servo2.set(targetPos);
        updateTelemetry();
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------TURRET--------------");
        telemetry.addData("State", currentState);
        telemetry.addData("Angle to Goal", angleToGoal);
        telemetry.addData("Target Degrees", targetDeg);
        telemetry.addData("Target Pos", targetPos);
        telemetry.addData("Analog Angle", encoderAngle);
    }

    @Override
    public void stop() {

    }

    public void setIdle() {
        currentState = TurretState.IDLE;
    }

    public void setAiming() {
        currentState = TurretState.AIMING;
    }

    public void setPreset() {
        currentState = TurretState.PRESET;
    }

    private double angleToPos(double turretDeg) {
        if (Double.isNaN(turretDeg)) {
            return Double.NaN;
        }
        if (turretDeg < MIN_TURRET_ANGLE_LIMIT) turretDeg = MIN_TURRET_ANGLE_LIMIT;
        if (turretDeg > MAX_TURRET_ANGLE_LIMIT) turretDeg = MAX_TURRET_ANGLE_LIMIT;
        return RIGHT_TURRET_POS
                + (turretDeg - RIGHT_TURRET_ANGLE) * (LEFT_TURRET_POS - RIGHT_TURRET_POS)
                / (LEFT_TURRET_ANGLE - RIGHT_TURRET_ANGLE);
    }

    public boolean isAligned() {
//        double targetAngle = Math.toDegrees(robotState.getVectorToGoal().getTheta() - robotState.getPose().getHeading() + Math.PI);
//        targetAngle = MathUtils.normalizeDegrees(targetAngle, false);
//        return Math.abs(encoderAngle - targetAngle) < 5;
        return true;
    }
}