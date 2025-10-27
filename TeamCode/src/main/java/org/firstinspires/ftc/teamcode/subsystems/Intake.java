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

    private boolean active = false;

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
    }

    @Override
    public void run() {
        if (active){
            motor.set(1);
        }else {
            motor.set(0.0);
        }
        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        telemetry.addLine("--------------INTAKE--------------");
        telemetry.addData("Active", active);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public void runIntake() {
        // Only spin intake if the spindexer is ready to receive
        active = robotState.isSpindexerAlignedForIntake();
    }

    public void stopIntake() {
        active = false;
    }
}