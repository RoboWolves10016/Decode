package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.hardware.motors.Motor.RunMode;

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
//    private final Debouncer debouncer = new Debouncer(0.05, Debouncer.DebounceType.Rising);
    private boolean debouncedSensorTripped;

    private ServoEx rightLight;

    public static double holdKP = 0.001;

    public static double holdPower = -0.22;
    public static double indexPower = -0.4;
    public static double closeFeedPower = 0.8;
    public static double farFeedPower = 0.4;

    public static boolean useManualOverride = false;
    public static double manualOverrideThrottle = 0.0;
    private boolean forceFeed = false;
//    private PIDFController controller;

    public static double motorHoldPos = 0.0;

    private final ElapsedTime indexTimer = new ElapsedTime();
    private double dutyCycle = 0.0;

    public enum IndexerWantedState {
        IDLE,
        INTAKE,
        EXHAUST,
        LAUNCH
    }

    private enum IndexerState {
        IDLE(holdPower),
        INTAKE(-0.4),
        HAS_2(holdPower),
        INDEX(indexPower),
        FULL(holdPower),
//        FEED(closeFeedPower),
        FEED(1.0),
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

        rightLight = new ServoEx(hwMap, "RightLight");
    }

    @Override
    public void run() {
        sensorTripped = !sensor.getState(); // Returns whether there is an object blocking the beam
//        debouncedSensorTripped = debouncer.calculate(sensorTripped);
        debouncedSensorTripped = sensorTripped;
        rightLight.set(debouncedSensorTripped ? 0.5 : 0.0);

        if (robotState.getPose().getY() < 72) IndexerState.FEED.dutyCycle = farFeedPower;
        else IndexerState.FEED.dutyCycle = closeFeedPower;

        previousState = currentState;
        if (wantedState == IndexerWantedState.EXHAUST) {
            currentState = IndexerState.EXHAUST;
            robotState.setIndexerLoaded(false);
            robotState.setHas3Balls(false);
        } else if (wantedState == IndexerWantedState.LAUNCH && (forceFeed || (
                robotState.isLauncherReady()
                && robotState.getVelocity().getMagnitude() < 7))) {
            currentState = IndexerState.FEED;
            robotState.setIndexerLoaded(false);
            robotState.setHas3Balls(false);
        } else if (wantedState == IndexerWantedState.INTAKE) {
              currentState = IndexerState.INTAKE;
        } else switch (currentState) {
            case IDLE:
            case INTAKE:
                if (debouncedSensorTripped) {
                    currentState = IndexerState.HAS_2;
                    robotState.setIndexerLoaded(true);
                }
                break;
            case HAS_2:
                if (!debouncedSensorTripped) {
                    currentState = IndexerState.IDLE;
                } else if (robotState.isIntakeFull()) {
                    currentState = IndexerState.INDEX;
                    indexTimer.reset();
                }
                break;
            case INDEX:
                if (indexTimer.seconds() > LauncherConstants.INDEX_TIME) {
                    robotState.setHas3Balls(true);
                    currentState = IndexerState.FULL;
                }
            case FULL:

                break; // Remain in the full state
            case FEED:
                if (wantedState != IndexerWantedState.LAUNCH) currentState = IndexerState.IDLE;
            default:
                if (wantedState == IndexerWantedState.IDLE ) currentState = IndexerState.IDLE;
                break;
        }

        // Write outputs
        dutyCycle = currentState.dutyCycle;

        if (useManualOverride) dutyCycle = manualOverrideThrottle;

        motor.set(dutyCycle);

        updateTelemetry();
    }

    public void forceFeed() {
        forceFeed = true;
    }

    public void stopForceFeed() {
        forceFeed = false;
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------INDEXER--------------");
        telemetry.addData("Wanted State", wantedState.toString());
        telemetry.addData("Current State", currentState.toString());
        telemetry.addData("Duty Cycle", dutyCycle);
        telemetry.addData("Current Position", motor.getCurrentPosition());
        telemetry.addData("Motor RunMode", motor.motorEx.getMode().toString());
        telemetry.addData("Sensor Tripped", debouncedSensorTripped);
        telemetry.addData("Cached Hold Pos", motorHoldPos);
    }

    @Override
    void stop() {
        motor.stopMotor();
    }
}
