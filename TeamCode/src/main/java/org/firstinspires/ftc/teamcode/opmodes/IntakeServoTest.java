package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

@Configurable
@TeleOp(name="Servo Test", group="Test")
public class IntakeServoTest extends OpMode {

    private MotorEx intakeMotor;
    private MotorEx rampMotor;
    private ServoEx servo;
    public static double setpoint = 0.3;
    public static double intakeSpeed = 0.0;
    public static double rampSpeed = 0.0;


    @Override
    public void init() {
        intakeMotor = new MotorEx(hardwareMap, "IntakeMotor");
        intakeMotor.setInverted(false);
        intakeMotor.setCachingTolerance(0.01);

        rampMotor = new MotorEx(hardwareMap, "RampMotor");
        rampMotor.setInverted(false);
        rampMotor.setCachingTolerance(0.01);

        servo = new ServoEx(hardwareMap, "Servo");
        servo.setInverted(false);
        servo.setCachingTolerance(0.01);
    }

    @Override
    public void loop() {
        servo.set(setpoint);
        intakeMotor.set(intakeSpeed);
        rampMotor.set(rampSpeed);

        telemetry.addData("Servo Pos",setpoint);
        telemetry.addData("Ramp Speed",rampSpeed);
        telemetry.addData("Intake Speed",intakeSpeed);

    }
}
