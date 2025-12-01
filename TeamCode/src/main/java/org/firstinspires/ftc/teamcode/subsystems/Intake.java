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
        IDLE(0),
        WAIT(0.4),
        INTAKE(1.0),
        FULL(-0.5);

        public final double speed;
        private IntakeState(double speed) {
            this.speed = speed;
        }
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
        motor.set(currentState.speed);

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

    public void stopIntake() {
        currentState = IntakeState.IDLE;
    }
}