package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotState;

public class Turret extends Subsystem{

    private static final double GEARBOX_RATIO = (40d / 20d) * (48d / 20d);
    private static final double TURRET_RATIO = GEARBOX_RATIO * (22d / 101d);
    private static final double MAX_ANGLE = TURRET_RATIO * 360d;
    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 1.0;
    private static final double CENTER_POS = 0.5;

    // Required subsystem components
    private final TelemetryManager telemetry;
    private final HardwareMap hwMap;
    private final RobotState robotState = RobotState.getInstance();

    private double angleToGoal = 0;
    private double servoTarget = 0;
    private double servoAngle = 0;

    private ServoExGroup turretServos;
    private ServoEx servo1;
    private AbsoluteAnalogEncoder servo1Encoder;

    private ServoEx servo2;

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
    void init() {
        servo1 = new ServoEx(hwMap, "TurretServo1");
        servo2 = new ServoEx(hwMap, "TurretServo2");

        servo1Encoder = new AbsoluteAnalogEncoder(hwMap, "TurretServo1Encoder", 3.3, AngleUnit.DEGREES);

        turretServos = new ServoExGroup(servo1, servo2);
        turretServos.setCachingTolerance(0.005);

        // Configure such that increasing the position rotates the turret counterclockwise
        turretServos.setInverted(true);
    }

    @Override
    void run() {
        servoAngle = servo1Encoder.getCurrentPosition();
        angleToGoal = robotState.getVectorToGoal().getTheta() - robotState.getPose().getHeading();

        // The following loop ensures that whatever angle we use is between -180 and 180 degrees
        while (angleToGoal < -Math.PI || angleToGoal > Math.PI) {
            if (angleToGoal > Math.PI) angleToGoal -= 2 * Math.PI;
            if (angleToGoal < -Math.PI) angleToGoal += 2 * Math.PI;
        }

        double targetPos = radiansToPos(Math.toDegrees(angleToGoal));

        switch (currentState) {
            case IDLE:
                servoTarget = CENTER_POS;
                break;
            case AIMING:
                servoTarget = targetPos;
                break;
        }
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------TURRET--------------");
        telemetry.addData("State", currentState);
        telemetry.addData("Angle to Goal", angleToGoal);
        telemetry.addData("Current Angle", servoAngle);
        telemetry.addData("Current Target", servoTarget);
    }

    @Override
    void stop() {
    }

    // Provided radians should already be normalized between -pi and pi
    public static double radiansToPos(double radians) {
        if (Double.isNaN(radians)) {
            return Double.NaN;
        }

        return 0.5 + 0.5;
    }

    public static double posToRadians(double pos) {
        if (Double.isNaN(pos)) {
            return Double.NaN;
        }

        return 0;
    }

}
