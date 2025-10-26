package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotState;

@Configurable
public class Spindexer extends Subsystem {

    private HardwareMap hwMap;
    private final TelemetryManager telemetry;
    private final RobotState robotState;
    private CRServoEx servo;
    private AbsoluteAnalogEncoder encoder;
    private final PIDFController positionController = new PIDFController(new PIDFCoefficients(kP, kI, kD, 0));
    public static double setpointDegrees = 0;
    private int selectedSlot = 0;
    private static final double ENCODER_OFFSET = -79;
    private enum SpindexerState {
        INTAKE,
        LAUNCH;
    }

    private SpindexerState state = SpindexerState.LAUNCH;


    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    private double currentPosition;

    public Spindexer(HardwareMap hwMap ) {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.hwMap = hwMap;
        this.robotState = RobotState.getInstance();
    }

    @Override
    public void init() {
        encoder = new AbsoluteAnalogEncoder(hwMap, "Analog", 3.3, AngleUnit.DEGREES);
        encoder.setReversed(false);
        servo = new CRServoEx(
                hwMap,
                "Spindexer",
                encoder,
                CRServoEx.RunMode.RawPower);

        servo.getServo().getController().pwmEnable();
        positionController.setPIDF(kP, kI, kD, 0);
    }

    @Override
    public void run() {
        selectedSlot = selectedSlot % 3;

        currentPosition = encoder.getCurrentPosition() % 360;
        positionController.setPIDF(kP, kI, kD, 0);

//        setpointDegrees = + 120 * selectedSlot;

        servo.set(positionController.calculate(currentPosition, setpointDegrees + ENCODER_OFFSET));
//        servo.set((setpointDegrees % 360) + ENCODER_OFFSET);
        updateTelemetry();

    }

    public void setTarget(double setpoint) {
        setpointDegrees = setpoint;
    }

    @Override
    protected void updateTelemetry(){
        telemetry.addLine("--------------SPINDEXER--------------");
        telemetry.addData("Current Position", currentPosition - ENCODER_OFFSET);
        telemetry.addData("Setpoint Degrees", setpointDegrees);
        telemetry.addData("Selected Slot", selectedSlot);
    }

    @Override
    public void stop() {

    }

    public void stepClockwise() {
        selectedSlot = selectedSlot + 1;
    }

    public void stepCounterClockwise() {
        selectedSlot = selectedSlot - 1;
    }

}
