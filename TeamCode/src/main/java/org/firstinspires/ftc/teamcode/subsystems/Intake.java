package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.RobotState;

public class Intake extends Subsystem {

    private final TelemetryManager telemetry;
    private final RobotState robotState;
    private MotorEx intakeMotor;
    private MotorEx rampMotor;
    private ServoEx servo;
    private final HardwareMap hwMap;

    private static final double IDLE_POS = 0.6;
    private static final double INTAKE_POS = 0.38;

    private enum IntakeState {
        IDLE(0, IDLE_POS),
        INTAKE(1.0, INTAKE_POS),
        SAFE_INTAKE(1.0, IDLE_POS),
        EXHAUST(-0.8, IDLE_POS);

        public final double speed;
        public final double pos;
        IntakeState(double speed, double pos) {
            this.speed = speed;
            this.pos = pos;
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
        servo = new ServoEx(hwMap, "IntakeServo");
        servo.setCachingTolerance(0.01);

        intakeMotor = new MotorEx(hwMap, "Intake");
        intakeMotor.setInverted(true);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setCachingTolerance(0.01);
    }

    @Override
    public void run() {
        intakeMotor.set(currentState.speed);
        servo.set(currentState.pos);
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
}