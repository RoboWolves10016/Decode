package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotState;

public class Intake extends Subsystem {

    private final TelemetryManager telemetry;
    private final RobotState robotState;
    private MotorEx motor;
    private final HardwareMap hwMap;

    private enum IntakeState {
        IDLE,
        WAIT,
        FAST_WAIT,
        INTAKE,
        FULL
    }

    private IntakeState currentState = IntakeState.IDLE;

    public Intake(HardwareMap ThisIsASentence) {
        this.hwMap = ThisIsASentence;
        this.robotState = RobotState.getInstance();
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init() {
        motor = new MotorEx(hwMap, "Intake");
        motor.setInverted(true);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setCachingTolerance(0.01);
    }

    @Override
    public void run() {
        switch(currentState) {
            case IDLE:
                motor.set(0);
                break;
            case INTAKE:
                motor.set(1);
                break;
            case WAIT:
                motor.set(0.3);
                break;
            case FAST_WAIT:
                motor.set(0.4);
                break;
            case FULL:
                motor.set(-0.3);
                break;
        }

        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        telemetry.addLine("--------------INTAKE--------------");
        telemetry.addData("State", currentState);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public void runIntake() {

        if (robotState.isFull()) {
            currentState = IntakeState.FULL;
            return;
        }
        // Only spin intake if the spindexer is ready to receive
        if (robotState.isSpindexerAlignedForIntake()) {
            currentState = IntakeState.INTAKE;
        } else {
            currentState = IntakeState.WAIT;
        }
    }

    public void runIntakeAuton() {
        if (robotState.isSpindexerAlignedForIntake()) {
            currentState = IntakeState.INTAKE;
        } else {
            currentState = IntakeState.FAST_WAIT;
        }
    }

    public void stopIntake() {
        currentState = IntakeState.IDLE;
    }
}