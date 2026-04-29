package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.Debouncer;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotState;

import lombok.Setter;

@Configurable
public class Intake extends Subsystem {

    private final TelemetryManager telemetry;
    private final HardwareMap hwMap;
    private final RobotState robotState;
    private MotorEx intakeMotor;
    private ServoEx servo;

    private Debouncer currentDebouncer = new Debouncer(0.3);


    private static final double MAX_POS = 0.92;
    private static final double MIN_POS = 0.0;
    private static final double IDLE_POS = 0.6;
    private static final double INTAKE_POS = 0.39;

    public static double HOLD_POS = 0.1;
    public static double FEED_POS = 0.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double EXHAUST_POWER = -0.5;
    private static final double FEED_POWER = 1.0;
    public static boolean manualOverride = false;
    public static double manualPos = IDLE_POS;

    private double posTweak = 0.00;
    private boolean isOverCurrent = false;

    private enum IntakeState {
        IDLE(0, IDLE_POS),
        INTAKE(INTAKE_POWER, INTAKE_POS),
        FULL(0.15, HOLD_POS),
        EXHAUST(EXHAUST_POWER, IDLE_POS),
        FEED(FEED_POWER, INTAKE_POS);

        public final double speed;
        public final double pos;
        IntakeState(double speed, double pos) {
            this.speed = speed;
            this.pos = pos;
        }
    }

    public enum IntakeWantedState {
        IDLE,
        INTAKE,
        EXHAUST,
        LAUNCH
    }

    private IntakeState currentState = IntakeState.IDLE;
    @Setter
    private IntakeWantedState wantedState = IntakeWantedState.IDLE;

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
        intakeMotor.setCurrentAlert(6, CurrentUnit.AMPS);

    }

    @Override
    public void run() {
        isOverCurrent = currentDebouncer.calculate(intakeMotor.isOverCurrent());

        switch (currentState) {
            case IDLE:
                currentState = handleIdle(wantedState);
                break;
            case INTAKE:
                currentState = handleIntake(wantedState);
                break;
            case EXHAUST:
                currentState = handleExhaust(wantedState);
                break;
            case FEED:
                currentState = handleFeed(wantedState);
                break;
            case FULL:
                currentState = handleFull(wantedState);
                break;
        }

        // Set outputs
        intakeMotor.set(currentState.speed);
        servo.set(MathUtils.clamp(manualOverride ? manualPos : currentState.pos + posTweak, MIN_POS, MAX_POS));
        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        telemetry.addLine("--------------INTAKE--------------");
        telemetry.addData("State", currentState);
        telemetry.addData("Pos", currentState.pos);
        telemetry.addData("Pos Tweak", posTweak);
        telemetry.addData("Speed", currentState.speed);
        telemetry.addData("Above 6A?", isOverCurrent);
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }

    private IntakeState handleIdle(IntakeWantedState ws) {
        switch (ws) {
            case LAUNCH: return robotState.isLauncherReady() ? IntakeState.FEED : IntakeState.IDLE;
            case INTAKE: return IntakeState.INTAKE;
            case EXHAUST: return IntakeState.EXHAUST;
            default: return IntakeState.IDLE;
        }
    }

    private IntakeState handleIntake(IntakeWantedState ws) {
        switch (ws) {
            case INTAKE: return IntakeState.INTAKE;
            case EXHAUST: return IntakeState.EXHAUST;
            case IDLE: return IntakeState.IDLE;
            case LAUNCH: return robotState.isLauncherReady() ? IntakeState.FEED : IntakeState.INTAKE;
        }
        return IntakeState.INTAKE; // unreachable
    }

    private IntakeState handleFull(IntakeWantedState ws) {
        switch (ws) {
            case EXHAUST: return IntakeState.EXHAUST;
            case LAUNCH: return robotState.isLauncherReady() ? IntakeState.FEED : IntakeState.FULL;
            default: return IntakeState.FULL;
        }
    }

    private IntakeState handleExhaust(IntakeWantedState ws) {
        switch (ws) {
            case INTAKE: return IntakeState.INTAKE;
            case EXHAUST: return IntakeState.EXHAUST;
            case LAUNCH: return robotState.isLauncherReady() ? IntakeState.FEED : IntakeState.EXHAUST;
            default: return IntakeState.IDLE;
        }
    }

    private IntakeState handleFeed(IntakeWantedState ws) {
        switch (ws) {
            case IDLE: return IntakeState.IDLE;
            case LAUNCH: return IntakeState.FEED;
            case INTAKE: return IntakeState.INTAKE;
            case EXHAUST: return IntakeState.EXHAUST;
        }
        return IntakeState.FEED; // unreachable
    }


    public void tweakUp() {
        posTweak += 0.01;
    }

    public void tweakDown() {
        posTweak -= 0.01;
    }
}