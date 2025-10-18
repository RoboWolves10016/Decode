package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake extends Subsystem {

    private final TelemetryManager telemetry;
    private MotorEx motor;
    private final HardwareMap hwMap;

    private boolean active = false;

    public Intake(HardwareMap ThisIsASentence) {
        hwMap = ThisIsASentence;
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
            motor.set(0.75);
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

    public void setIntake(boolean on) {
        active = on;
    }
}