package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.util.SpindexerSlot;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import lombok.var;

@Configurable
public class Spindexer extends Subsystem {

    private HardwareMap hwMap;
    private final TelemetryManager telemetry;
    private final RobotState robotState;
    private ServoEx servo;
    private ColorSensors colorSensors;
    private AbsoluteAnalogEncoder encoder;
    private double setpoint = 0;
    private SpindexerSlot currentSlot = SpindexerSlot.ONE;
    private final ElapsedTime stepTimer = new ElapsedTime();

    private final Map<SpindexerSlot, ColorSensors.BallState> map = new HashMap<>();
    private enum SpindexerState {
        INTAKE,
        LAUNCH
    }

    private SpindexerState state = SpindexerState.LAUNCH;

    private double currentPosition;

    public Spindexer(HardwareMap hwMap ) {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.hwMap = hwMap;
        this.robotState = RobotState.getInstance();
        this.colorSensors = new ColorSensors(hwMap);
        map.put(SpindexerSlot.ONE, ColorSensors.BallState.EMPTY);
        map.put(SpindexerSlot.TWO, ColorSensors.BallState.EMPTY);
        map.put(SpindexerSlot.THREE, ColorSensors.BallState.EMPTY);
    }

    @Override
    public void init() {
        encoder = new AbsoluteAnalogEncoder(hwMap, "Analog", 3.3, AngleUnit.DEGREES);
        encoder.setReversed(false);
        servo = new ServoEx(
                hwMap,
                "Spindexer");
        colorSensors.init();

        servo.getServo().getController().pwmEnable();
    }

    @Override
    public void run() {
        colorSensors.run();
        currentPosition = encoder.getCurrentPosition();
        robotState.setCurrentSlot(currentSlot);
        robotState.setCurrentSlotBallState(map.get(currentSlot));
        if (state == SpindexerState.LAUNCH) {
            // LAUNCH STATE CODE
            setpoint = currentSlot.launchPosition;
//            setSlotData(robotState.getCurrentSlot(), robotState.getCurrentSlotBallState());
        } else {
            // INTAKE STATE CODE
            setpoint = currentSlot.intakePosition;

//            if (robotState.isSpindexerAlignedForIntake()
//                    && colorSensors.getCurrentStateDuration() > 0.12)
//            {
//                setSlotData(currentSlot, colorSensors.getCurrentBallState());
//                if (colorSensors.getCurrentBallState() != ColorSensors.BallState.EMPTY && !isFull()) {
//                    stepClockwise();
//                }
//            }
        }
        servo.set(setpoint);

        // Update Robot State safety variables
        robotState.setSpindexerAlignedForLaunch(
                Math.abs(currentPosition  - currentSlot.launchMeasurement)
                        < Tuning.SPINDEXER_ALIGNED_TOLERANCE_DEG);
        robotState.setSpindexerAlignedForIntake(
                Math.abs(currentPosition - currentSlot.intakeMeasurement)
                        < Tuning.SPINDEXER_ALIGNED_TOLERANCE_DEG);

        updateTelemetry();

    }

    @Override
    protected void updateTelemetry(){
        telemetry.addLine("--------------SPINDEXER--------------");
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Servo Setpoint", setpoint);
        telemetry.addData("Selected Slot", currentSlot);
        telemetry.addData("State", state);
        telemetry.addData("Slot 1 State", Objects.requireNonNull(map.get(SpindexerSlot.ONE)));
        telemetry.addData("Slot 2 State", Objects.requireNonNull(map.get(SpindexerSlot.TWO)));
        telemetry.addData("Slot 3 State", Objects.requireNonNull(map.get(SpindexerSlot.THREE)));
        colorSensors.updateTelemetry();
    }

    @Override
    public void stop() {

    }

    public void setLaunchMode() {
        state = SpindexerState.LAUNCH;
    }

    public void setIntakeMode() {
        state = SpindexerState.INTAKE;
    }

    public void stepClockwise() {
        if (stepTimer.seconds() > 0.5) {
            currentSlot = currentSlot.last();
            stepTimer.reset();
        }
    }

    public void stepCounterClockwise() {
        if (stepTimer.seconds() > 0.5) {
            currentSlot = currentSlot.next();
            stepTimer.reset();
        }
    }

    private boolean isFull() {
        return !map.containsValue(ColorSensors.BallState.EMPTY);
    }

    public void setSlotData(SpindexerSlot slot, ColorSensors.BallState ballState) {
        map.put(slot, ballState);
    }
}
