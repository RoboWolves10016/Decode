package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Interpolation;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.MathUtils;

import lombok.Setter;

@Configurable
public class Flywheel extends Subsystem {

    // Required subsystem components
    private final TelemetryManager telemetry;
    private final HardwareMap hwMap;
    private final RobotState robotState = RobotState.getInstance();

    private MotorEx motor1;
    private MotorEx motor2;
    private MotorGroup motors;

    private final PIDFController velocityController = new PIDFController(0, 0, 0, 0);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    public static boolean useManualRpm = false;
    public static boolean useManualDutyCycle = false;
    public static double manualDutyCycle = 0.0;
    public static double manualRpm = 0;
    public static double additionalRpm = 0;
    public static boolean useIdleRpm = false;
    private double targetRpm = 0;
    private double currentRpm = 0;
    private double currentAccel = 0;
    private double distanceToGoal = 0;

//    public static double kP = 0.002;
//    public static double kI = 0.005;
//    public static double kD = 0;
//    public static double kS = 0.13;
//    public static double kV = 0.0002;
//    public static double kA  = 0.002;
    public static double kP = 0.008;
    public static double kI = 0.035;
    public static double kD = 0;
    public static double kS = 0.08;
    public static double kV = 0.000140;
    public static double kA = 0;

    public enum FlywheelState {
        IDLE,
        AUTO,
        PRESET
    }

    private FlywheelState state = FlywheelState.IDLE;

    public Flywheel(HardwareMap hwMap) {
        this.hwMap = hwMap;
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init() {
        motor1 = new MotorEx(hwMap, "Launcher1", Motor.GoBILDA.BARE);
        motor1.setInverted(true);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setRunMode(Motor.RunMode.RawPower);

        motor2 = new MotorEx(hwMap, "Launcher2", Motor.GoBILDA.BARE);
        motor2.setInverted(false);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor2.setRunMode(Motor.RunMode.RawPower);

        motors = new MotorGroup(motor1, motor2);
        motors.setRunMode(Motor.RunMode.RawPower);
        velocityController.setPIDF(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void run() {
        distanceToGoal = robotState.getVectorToGoal().getMagnitude();
        switch (state) {
            case IDLE:
                if (useIdleRpm) targetRpm = LauncherConstants.SHOT_SPEEDS[LauncherConstants.SHOT_SPEEDS.length / 2];
                else targetRpm = 0;
                break;
            case AUTO:
                targetRpm = distanceToRpm(distanceToGoal) + additionalRpm;
                if (robotState.getVectorToGoal().getMagnitude() < 56) targetRpm -= 100;
                break;
            case PRESET:
                if ((robotState.getPose().getX() > 72 && robotState.getAlliance() == Alliance.BLUE)
                        || (robotState.getPose().getX() < 72 && robotState.getAlliance() == Alliance.RED)) {
                    targetRpm = LauncherConstants.PRESET_RPM_FAR;
                } else {
                    targetRpm = LauncherConstants.PRESET_RPM_CLOSE;
                }
                break;
        }

        if (useManualRpm) targetRpm = manualRpm;

        currentAccel = motor1.getAcceleration();
        currentRpm = (motor1.getCorrectedVelocity() / 28) * 60;


        double output = MathUtils.clamp(velocityController.calculate(currentRpm, targetRpm) + feedforward.calculate(targetRpm, motor1.getAcceleration()), 0.0, 1.0);
        if (useManualDutyCycle) output = manualDutyCycle;
        telemetry.addData("MotorOutput", output);
        motors.set(output);

//        robotState.setLauncherReady(
//                Math.abs(currentRpm - targetRpm) < 65
//                        && state != FlywheelState.IDLE);
    }

    @Override
    public void updateTelemetry() {
        telemetry.addLine("--------------FLYWHEEL--------------");
        telemetry.addData("Distance to Goal", distanceToGoal);
        telemetry.addData("Current RPM", currentRpm);
        telemetry.addData("Current Accel", currentAccel);
        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("State", state);

    }

    @Override
    public void stop() {
        motor1.stopMotor();
    }

    private double distanceToRpm(double distanceInches) {
        return Interpolation.interpolate(LauncherConstants.SHOT_DISTANCES, LauncherConstants.SHOT_SPEEDS, distanceInches);
    }

    public void setAuto() {
        state = FlywheelState.AUTO;
    }

    public void setIdle() {
        state = FlywheelState.IDLE;
    }

    public void setPreset() {
        state = FlywheelState.PRESET;
    }

    public boolean isReady() {
        return Math.abs(currentRpm - targetRpm) < 100;
    }

}
