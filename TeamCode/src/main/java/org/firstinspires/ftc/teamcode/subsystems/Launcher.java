package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Interpolation;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import java.util.function.Supplier;

@Configurable
public class Launcher extends Subsystem {

    // Required subsystem components
    private final TelemetryManager telemetry;
    private final HardwareMap hwMap;
    private final RobotState robotState = RobotState.getInstance();

    private MotorEx motor;

    private final PIDFController velocityController = new PIDFController(0, 0, 0, 0);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    public static double targetRpm = 0;
    private double currentRpm = 0;
    private double distanceToGoal = 0;

    public static double kP = 0.00003;
    public static double kI = 200.0;
    public static double kD = 0.000001;
    public static double kS = 0;
    public static double kV = 0.00019;
    public static double kA = 0;

    public Launcher(HardwareMap hwMap) {
        this.hwMap = hwMap;
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init() {
        motor = new MotorEx(hwMap, "Launcher", Motor.GoBILDA.BARE);
        motor.setInverted(true);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(kP, kI, kD);
        motor.setFeedforwardCoefficients(0, kV, 0);
        velocityController.setPIDF(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    }

    @Override
    public void run() {
        motor.setVeloCoefficients(kP, kI, kD);
        motor.setFeedforwardCoefficients(kS, kV, kA);
        distanceToGoal = robotState.getVectorToGoal().getMagnitude();
//        targetRpm = distanceToRpm(distanceToGoal);
        currentRpm = (motor.getCorrectedVelocity() / 28) * 60;
        motor.set(targetRpm);

        velocityController.setPIDF(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        motor.set(velocityController.calculate(currentRpm, targetRpm) + feedforward.calculate(currentRpm, motor.getAcceleration()));
        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        telemetry.addLine("--------------LAUNCHER--------------");
        telemetry.addData("Distance to Goal", distanceToGoal);
        telemetry.addData("Current RPM", currentRpm);
        telemetry.addData("Target RPM", targetRpm);

    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    private double distanceToRpm(double distanceInches) {
        return Interpolation.interpolate(Tuning.DISTANCED_FROM_GOAL_FEET, Tuning.REVOLUTIONS_PER_MINUTE, distanceInches / 12);
    }

    public boolean isReady() {
        return Math.abs(currentRpm - targetRpm) <= 50;
    }
}
