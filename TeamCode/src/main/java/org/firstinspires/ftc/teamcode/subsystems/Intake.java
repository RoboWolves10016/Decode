package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private DigitalChannel sensor;
    private ServoEx leftLight;
    private boolean sensorTripped = false;
    private final Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.Rising);
    private boolean debouncedSensorTripped;

    private static final double MAX_POS = 0.92;
    private static final double MIN_POS = 0.0;
    private static final double IDLE_POS = 0.65;
    private static final double INTAKE_POS = 0.5;

//    public static double HOLD_POS = 0.1;
    public static double HOLD_POS = 0.3;
    public static double FEED_POS = 0.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double EXHAUST_POWER = -0.5;
    private static final double FEED_POWER = 1.0;
    public static boolean manualServoOverride = false;
    public static double manualPos = IDLE_POS;
    public static boolean useManualPower = false;
    public static double manualPower = 0.0;


    private double posTweak = 0.00;
    private boolean isOverCurrent = false;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private enum IntakeState {
        IDLE(0, IDLE_POS),
        INTAKE(INTAKE_POWER, INTAKE_POS),
        INDEX(-0.2, HOLD_POS),
        FULL(0, HOLD_POS),
        EXHAUST(EXHAUST_POWER, IDLE_POS),
        FEED(FEED_POWER, 0.18);

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

        sensor = hwMap.get(DigitalChannel.class, "IntakeSensor");

        leftLight = new ServoEx(hwMap, "LeftLight");
    }

    @Override
    public void run() {
        sensorTripped = !sensor.getState();
        debouncedSensorTripped = debouncer.calculate(sensorTripped);
        IntakeState lastState = currentState;

        switch (currentState) {
            case IDLE:
                currentState = handleIdle(wantedState);
                break;
            case INTAKE:
                currentState = handleIntake(wantedState);
                break;
            case INDEX:
                currentState = handleIndex();
                break;
            case FULL:
                currentState = handleFull(wantedState);
                break;
            case EXHAUST:
                currentState = handleExhaust(wantedState);
                break;
            case FEED:
                currentState = handleFeed(wantedState);
                break;
        }
        if (currentState != lastState) stateTimer.reset();

        // Set outputs
        intakeMotor.set(useManualPower ? manualPower : currentState.speed);
        servo.set(MathUtils.clamp(manualServoOverride ? manualPos : currentState.pos + posTweak, MIN_POS, MAX_POS));
        leftLight.set(debouncedSensorTripped ? 0.5 : 0.0);
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
        telemetry.addData("Sensor Tripped", sensorTripped);
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
            case INTAKE:
                if (robotState.isIndexerLoaded() && debouncedSensorTripped) {
                    stateTimer.reset();
                    robotState.setIntakeFull(true);
                    return IntakeState.INDEX;
                }
                return IntakeState.INTAKE;
            case EXHAUST: return IntakeState.EXHAUST;
            case IDLE: return IntakeState.IDLE;
            case LAUNCH: return robotState.isLauncherReady() ? IntakeState.FEED : IntakeState.INTAKE;
        }
        return IntakeState.INTAKE; // unreachable
    }

    private IntakeState handleIndex() {
        if (!debouncedSensorTripped) return IntakeState.INTAKE;
        if (stateTimer.seconds() > LauncherConstants.INDEX_TIME) return IntakeState.FULL;
        return IntakeState.INDEX;
    }

    private IntakeState handleFull(IntakeWantedState ws) {
        if (!debouncedSensorTripped) return IntakeState.INTAKE;
        switch (ws) {
            case EXHAUST: return IntakeState.EXHAUST;
            case LAUNCH: return robotState.isLauncherReady() ? IntakeState.FEED : IntakeState.FULL;
            default: return IntakeState.FULL;
        }
    }

    private IntakeState handleExhaust(IntakeWantedState ws) {
        robotState.setIntakeFull(false);
        switch (ws) {
            case INTAKE: return IntakeState.INTAKE;
            case EXHAUST: return IntakeState.EXHAUST;
            case LAUNCH: return robotState.isLauncherReady() ? IntakeState.FEED : IntakeState.EXHAUST;
            default: return IntakeState.IDLE;
        }
    }

    private IntakeState handleFeed(IntakeWantedState ws) {
        robotState.setIntakeFull(false);
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