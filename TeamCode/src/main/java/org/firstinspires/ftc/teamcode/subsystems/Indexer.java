package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.hardware.motors.Motor.RunMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.Debouncer;

import org.firstinspires.ftc.teamcode.RobotState;

import lombok.Setter;

@Configurable
public class Indexer extends Subsystem{
    private final TelemetryManager telemetry;
    private final HardwareMap hwMap;
    private final RobotState robotState;

    private MotorEx motor;
    private DigitalChannel sensor;
    private boolean sensorTripped = false;
    private final Debouncer debouncer = new Debouncer(0.2, Debouncer.DebounceType.Falling);
    private boolean debouncedSensorTripped;

    public static double holdKP = 0.001;

    public static double intakePower = 0.3;
    public static double inchPower = 0.0;
    public static double holdPower = -0.22;
    public static double closeFeedPower = 0.8;
    public static double farFeedPower = 0.4;

    public static boolean useManualOverride = false;
    public static double manualOverrideThrottle = 0.0;
//    private PIDFController controller;

    public static double motorHoldPos = 0.0;
    private double motorThrottle = 0.0;

    public enum IndexerWantedState {
        IDLE,
        INTAKE,
        EXHAUST,
        LAUNCH
    }

    private enum IndexerState {
        IDLE(0.0), // 0
        INTAKE(intakePower), // 1
        INCH(holdPower),
        HOLD(holdPower),
        FEED(closeFeedPower),
        EXHAUST(-1.0);

        public double dutyCycle;

        IndexerState(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }
    }

    @Setter
    private IndexerWantedState wantedState = IndexerWantedState.IDLE;
    private IndexerState currentState = IndexerState.IDLE;
    private IndexerState previousState = IndexerState.IDLE;

    public Indexer(HardwareMap hwMap) {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.hwMap = hwMap;
        this.robotState = RobotState.getInstance();
//        this.controller = new PIDFController(0.005, 0, 0, 0.12)
    }

    @Override
    public void init() {
        motor = new MotorEx(hwMap, "RampMotor", Motor.GoBILDA.BARE);
        motor.setInverted(false);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setPositionCoefficient(holdKP); // TODO tune value
        motor.setRunMode(RunMode.RawPower);

        sensor = hwMap.get(DigitalChannel.class, "RampSensor");
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void run() {
        sensorTripped = !sensor.getState(); // Returns whether there is an object blocking the beam
        debouncedSensorTripped = debouncer.calculate(sensorTripped);

        if (robotState.getPose().getY() < 72) IndexerState.FEED.dutyCycle = farFeedPower;
        else IndexerState.FEED.dutyCycle = closeFeedPower;

        previousState = currentState;
        switch (currentState) {
            case INTAKE:
                currentState = handleIntake(wantedState);
                break;
            case INCH:
                currentState = handleInch(wantedState);
                break;
            case HOLD:
                currentState = handleHold(wantedState);
                break;
            default:
                currentState = handleDefault(wantedState);
                break;
        }

//         Write outputs
        motor.set(currentState.dutyCycle);

        updateTelemetry();
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------INDEXER--------------");
        telemetry.addData("Current State", currentState.toString());
        telemetry.addData("Duty Cycle", currentState.dutyCycle);
        telemetry.addData("Current Position", motor.getCurrentPosition());
        telemetry.addData("Motor RunMode", motor.motorEx.getMode().toString());
        telemetry.addData("Sensor Tripped", sensorTripped);
        telemetry.addData("Cached Hold Pos", motorHoldPos);
    }

    @Override
    void stop() {
        motor.stopMotor();
    }

    private IndexerState handleDefault(IndexerWantedState ws) {
        switch (ws) {
            case INTAKE: return IndexerState.INTAKE;
            case EXHAUST: return IndexerState.EXHAUST;
            case LAUNCH: return robotState.isLauncherReady() ? IndexerState.FEED : currentState;
            default: return IndexerState.IDLE;
        }
    }

    private IndexerState handleIntake(IndexerWantedState ws) {
        switch (ws) {
            case INTAKE: {
                if (debouncedSensorTripped) {
                    robotState.setIndexerLoaded(true);
                    return IndexerState.INCH;
                }
                return IndexerState.INTAKE;
            }
            case EXHAUST: return IndexerState.EXHAUST;
            case LAUNCH: return robotState.isLauncherReady() ? IndexerState.FEED : IndexerState.INTAKE;
            default: return IndexerState.IDLE;
        }
    }

    private IndexerState handleInch(IndexerWantedState ws) {
        if (ws == IndexerWantedState.LAUNCH)
            return robotState.isLauncherReady()
                ? IndexerState.FEED
                : IndexerState.INCH;
        if (ws == IndexerWantedState.EXHAUST) return IndexerState.EXHAUST;
        if (!debouncedSensorTripped) {
            // If sensor is no longer tripped, ball has reached hold position
            motorHoldPos = motor.getCurrentPosition();
            return IndexerState.HOLD;
        }
        return IndexerState.INCH;
    }

    private IndexerState handleHold(IndexerWantedState ws) {
        switch (ws) {
            case LAUNCH: {
                if (robotState.isLauncherReady()) {
                    robotState.setIndexerLoaded(false);
                    return IndexerState.FEED;
                }
                return IndexerState.HOLD;
            }
            case EXHAUST: return IndexerState.EXHAUST;
            default: return IndexerState.HOLD;
        }
    }

}
