package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.LauncherConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Tuning;

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
    private double targetAngle = 0;
    private double angleTarget = 0;
    private double encoderAngle = 0;

    private ServoExGroup turretServos;
    private ServoEx servo1;
    private ServoEx servo2;
    private AbsoluteAnalogEncoder encoder;

    @Setter
    private boolean tracking = false;

    private enum TurretState {
        AIMING,
        IDLE
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
        if (tracking) currentState = TurretState.AIMING;
        else currentState = TurretState.IDLE;
        if (Tuning.SHOOT_WHILE_MOVING)
            angleToGoal = robotState.getFutureVectorToGoal().getTheta() - robotState.getPose().getHeading() + Math.PI;
        else
            angleToGoal = robotState.getVectorToGoal().getTheta() - robotState.getPose().getHeading() + Math.PI;

        switch (currentState) {
            case IDLE:
                targetAngle = 0;
                break;
            case AIMING:
                targetAngle = Math.toDegrees(MathUtils.normalizeAngle(angleToGoal, false, AngleUnit.RADIANS));
                break;
        }

        if (useManualOverride) {
            angleTarget = manualOverrideDegrees;
        } else {
            angleTarget = targetAngle;
        }

        angleTarget -= TURRET_ROT_FF * robotState.getAngularVelocity();

        angleTarget = MathUtils.clamp(angleTarget, MIN_TURRET_ANGLE_LIMIT, MAX_TURRET_ANGLE_LIMIT);

        // Prevent limit wraparounds
        if (angleTarget > 0 && encoderAngle < 0 && Math.abs(angleTarget - encoderAngle) > 200) {
            angleTarget = -20;
        }

        if (angleTarget < 0 && encoderAngle > 0 && Math.abs(angleTarget - encoderAngle) > 200) {
            angleTarget = 20;
        }

        turretServos.set(angleToPos(angleTarget));
        updateTelemetry();
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------TURRET--------------");
        telemetry.addData("State", currentState);
        telemetry.addData("Angle to Goal", angleToGoal);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Target Pos", angleTarget);
        telemetry.addData("Analog Angle", encoderAngle);
    }

    @Override
    public void stop() {

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
        double targetAngle = Math.toDegrees(robotState.getVectorToGoal().getTheta() - robotState.getPose().getHeading() + Math.PI);
        targetAngle = MathUtils.normalizeDegrees(targetAngle, false);
        return Math.abs(encoderAngle - targetAngle) < 5;
    }
}