package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.util.SpindexerSlot;

import lombok.Getter;

@Configurable
public class Kicker extends Subsystem{

    private final TelemetryManager telemetry;
    private final HardwareMap hwMap;
    private final RobotState robotState;

    private ServoEx servo;
    private AbsoluteAnalogEncoder encoder;

    public static final double DOWN_POSITION = 0.4;
    public static final double UP_POSITION = 1.0;
    public static final double SAFE_THRESHOLD = 165;

    public static final double TOP_THRESHOLD = 175;

    private enum KickerState {
        IDLE,
        KICKING,
        RETURNING
    }

    // INSTANCE STATE VARIABLES
    private KickerState currentState = KickerState.IDLE;
    private KickerState lastState = KickerState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean feedBall = false;
    private double position;
    private double setpoint = DOWN_POSITION;

    private SpindexerSlot lastKickedSlot = null;


    public Kicker(HardwareMap hwMap) {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.robotState = RobotState.getInstance();
        this.hwMap = hwMap;
    }

    @Override
    public void init() {
        servo = new ServoEx(hwMap, "Kicker");
        encoder = new AbsoluteAnalogEncoder(hwMap, "KickerAnalog", 3.3, AngleUnit.DEGREES);
        servo.getServo().getController().pwmEnable();
    }

    @Override
    public void run() {
        position = encoder.getCurrentPosition();

        robotState.setKickerSafe(position < SAFE_THRESHOLD && currentState == KickerState.IDLE);

        // State transition logic
        switch (currentState) {
            case IDLE:
                runIdle();
                break;
            case KICKING:
                runKicking();
                break;
            case RETURNING:
                runReturning();
                break;
        }

        robotState.setBallKicked(currentState == KickerState.RETURNING);

        lastState = currentState;
        servo.set(setpoint);
        updateTelemetry();
    }

    private void runIdle() {
        setpoint = DOWN_POSITION;
        robotState.setKickerSafe(true);
        if (feedBall && robotState.isSpindexerAlignedForLaunch() && lastKickedSlot != robotState.getCurrentSlot()
        && robotState.isRpmReady()) {
            currentState = KickerState.KICKING;
        }
    }

    private void runKicking() {
        if (lastState == KickerState.IDLE) timer.reset();
        robotState.setKickerSafe(false);
        if (robotState.isSpindexerAlignedForLaunch()) {
            setpoint = UP_POSITION;
        } else {
            setpoint = DOWN_POSITION;
        }

        // State transition logic
        if (position > TOP_THRESHOLD || timer.seconds() > 0.5) {
            currentState = KickerState.RETURNING;
            lastKickedSlot = robotState.getCurrentSlot();
        }
    }

    private void runReturning() {
        robotState.setKickerSafe(false);
        setpoint = DOWN_POSITION;
        if (position < SAFE_THRESHOLD) currentState = KickerState.IDLE;
    }


    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------KICKER--------------");
        telemetry.addData("Position", position);
        telemetry.addData("Servo Setpoint", setpoint);
        telemetry.addData("Current State", currentState);
        telemetry.addData("Want Feed", feedBall);
    }

    @Override
    public void stop() {

    }

    public void feed() {
        feedBall = true;
    }

    public void stopFeed() {
        feedBall = false;
    }

    public void resetHistory() {
        lastKickedSlot = null;
    }
}
