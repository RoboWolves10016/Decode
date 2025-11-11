package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.util.Interpolation;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Launcher extends Subsystem {

    // Required subsystem components
    private final TelemetryManager telemetry;
    private final HardwareMap hwMap;
    private final RobotState robotState = RobotState.getInstance();

    private MotorEx motor;

    private final PIDFController velocityController = new PIDFController(0, 0, 0, 0);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    public static boolean useManualRpm = false;
    public static double targetRpm = 0;
    private double currentRpm = 0;
    private double distanceToGoal = 0;

    public static double kP = 0.002;
    public static double kI = 0.005;
    public static double kD = 0;
    public static double kS = 0.13;
    public static double kV = 0.0002;
    public static double kA = 0;

    public enum LauncherState {
        IDLE,
        AUTO,
        PRESET
    }

    private LauncherState state = LauncherState.IDLE;

    public Launcher(HardwareMap hwMap) {
        this.hwMap = hwMap;
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init() {
        motor = new MotorEx(hwMap, "Launcher", Motor.GoBILDA.BARE);
        motor.setInverted(true);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setRunMode(Motor.RunMode.RawPower);
        velocityController.setPIDF(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void run() {
        distanceToGoal = robotState.getVectorToGoal().getMagnitude();
        switch (state) {
            case IDLE:
                targetRpm = 0;
                break;
            case AUTO:
                if (!useManualRpm) {
                    targetRpm = distanceToRpm(distanceToGoal);
                }
                break;
            case PRESET:
                targetRpm = 3000;
                break;
        }

        currentRpm = (motor.getCorrectedVelocity() / 28) * 60;

//        velocityController.setPIDF(kP, kI, kD, 0);
//        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        motor.set(Math.max(0,velocityController.calculate(currentRpm, targetRpm) + feedforward.calculate(targetRpm, motor.getAcceleration())));

        robotState.setLauncherReady(Math.abs(currentRpm - targetRpm) < 50 && state != LauncherState.IDLE);
        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        telemetry.addLine("--------------LAUNCHER--------------");
        telemetry.addData("Distance to Goal", distanceToGoal);
        telemetry.addData("Current RPM", currentRpm);
        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("State", state);

    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    private double distanceToRpm(double distanceInches) {
        return Interpolation.interpolate(Tuning.DISTANCED_FROM_GOAL_INCHES, Tuning.REVOLUTIONS_PER_MINUTE, distanceInches);
    }

    public void setAuto() {
        state = LauncherState.AUTO;
    }

    public void setIdle() {
        state = LauncherState.IDLE;
    }
    public void setPreset() {
        state = LauncherState.PRESET;
    }

    public boolean isReady() {
        return Math.abs(currentRpm - targetRpm) <= 50;
    }
}
