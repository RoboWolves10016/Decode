package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.teamcode.RobotState;

@Configurable
public class Turret extends Subsystem{

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
    private double servoTarget = 0;
//    private double servoAngle = 0;

    private ServoExGroup turretServos;
    private ServoEx servo1;
    private ServoEx servo2;
//    private AbsoluteAnalogEncoder servo1Encoder;


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

//        servo1Encoder = new AbsoluteAnalogEncoder(hwMap, "TurretServo1Encoder", 3.3, AngleUnit.DEGREES);

        turretServos = new ServoExGroup(servo1, servo2);
        turretServos.setCachingTolerance(
                CENTER_POS - radiansToPos(CENTER_ANGLE - Math.toRadians(TURRET_CACHING_TOL_DEG))
        );
    }

    @Override
    public void run() {
//        servoAngle = servo1Encoder.getCurrentPosition();
        angleToGoal = robotState.getVectorToGoal().getTheta() - robotState.getPose().getHeading();

        // The following loop ensures that whatever angle we use is between -180 and 180 degrees
        while (angleToGoal < -ShooterConstants.MIN_ANGLE_LIMIT || angleToGoal > ShooterConstants.MAX_ANGLE_LIMIT) {
            if (angleToGoal > Math.PI) angleToGoal -= 2 * Math.PI;
            if (angleToGoal < -Math.PI) angleToGoal += 2 * Math.PI;
        }

        switch (currentState) {
            case IDLE:
                targetAngle = CENTER_ANGLE;
                servoTarget = CENTER_POS;
                break;
            case AIMING:
                targetAngle = angleToGoal;
                servoTarget = radiansToPos(angleToGoal);
                break;
        }
        if (useManualOverride) {
            targetAngle = manualOverrideDegrees;
            servoTarget = radiansToPos(Math.toRadians(manualOverrideDegrees));
        }

        turretServos.set(servoTarget);
        updateTelemetry();
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------TURRET--------------");
        telemetry.addData("State", currentState);
        telemetry.addData("Angle to Goal", angleToGoal);
        telemetry.addData("Target Angle", targetAngle);
//        telemetry.addData("Current Angle", servoAngle);
        telemetry.addData("Target Pos", servoTarget);
    }

    @Override
    public void stop() {

    }

    // Provided radians should already be normalized between -pi and pi
    private static double radiansToPos(double radians) {
        if (Double.isNaN(radians)) {
            return Double.NaN;
        }

        return CENTER_POS + (radians - CENTER_ANGLE) / TURRET_RANGE;
    }

    private static double posToRadians(double pos) {
        if (Double.isNaN(pos)) {
            return Double.NaN;
        }

        return 0;
    }

}
