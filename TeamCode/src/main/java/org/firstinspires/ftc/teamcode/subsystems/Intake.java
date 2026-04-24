package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.RobotState;

@Configurable
public class Intake extends Subsystem {

    private final TelemetryManager telemetry;
    private final RobotState robotState;
    private MotorEx intakeMotor;
    private MotorEx rampMotor;
    private ServoEx servo;
    private final HardwareMap hwMap;


    private static final double MAX_POS = 0.92;
    private static final double MIN_POS = 0.0;
    private static final double IDLE_POS = 0.6;
    private static final double INTAKE_POS = 0.38;

    private static final double INTAKE_POWER = 1.0;
    private static final double EXHAUST_POWER = -0.5;
    public static boolean manualOverride = false;
    public static double manualPos = IDLE_POS;

    private double posTweak = 0.00;

    private enum IntakeState {
        IDLE(0, IDLE_POS),
        INTAKE(INTAKE_POWER, INTAKE_POS),
        SAFE_INTAKE(INTAKE_POWER, IDLE_POS),
        EXHAUST(EXHAUST_POWER, IDLE_POS);

        public final double speed;
        public final double pos;
        IntakeState(double speed, double pos) {
            this.speed = speed;
            this.pos = pos;
        }
    }

    private IntakeState currentState = IntakeState.IDLE;

    public Intake(HardwareMap hwMap) {
        this.hwMap = hwMap;
        this.robotState = RobotState.getInstance();
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init() {
        servo = new ServoEx(hwMap, "IntakeServo");
        servo.setCachingTolerance(0.01);

        intakeMotor = new MotorEx(hwMap, "IntakeMotor");
        intakeMotor.setInverted(true);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setCachingTolerance(0.01);

        rampMotor = new MotorEx(hwMap, "RampMotor");
        rampMotor.setInverted(false);
    }

    @Override
    public void run() {
        intakeMotor.set(currentState.speed);
        rampMotor.set(currentState.speed);
        servo.set(MathUtils.clamp(manualOverride ? manualPos : currentState.pos + posTweak, MIN_POS, MAX_POS));
        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        telemetry.addLine("--------------INTAKE--------------");
        telemetry.addData("State", currentState);
        telemetry.addData("Pos", currentState.pos);
        telemetry.addData("Speed", currentState.speed);
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }

    public void runIntake() {
        currentState = IntakeState.INTAKE;
        if (robotState.isFull()) currentState = IntakeState.IDLE;
    }

    public void runExhaust() {
        currentState = IntakeState.EXHAUST;
    }

    public void stopIntake() {
        currentState = IntakeState.IDLE;
    }

    public void tweakUp() {
        posTweak += 0.01;
    }

    public void tweakDown() {
        posTweak -= 0.01;
    }
}