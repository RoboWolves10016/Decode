package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class Spindexer extends Subsystem {

    private HardwareMap hwMap;
    private final TelemetryManager telemetry;
    private CRServoEx servo;
    public static double setpointDegrees = 0;

    public static double kP = 0;
    public static double kD = 0;

    public Spindexer(HardwareMap hwMap ) {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.hwMap = hwMap;
    }

    @Override
    public void init() {
        servo = new CRServoEx(
                hwMap,
                "Spindexer",
                "Analog",
                3.3,
                AngleUnit.DEGREES,
                CRServoEx.RunMode.OptimizedPositionalControl);

        servo.setPIDF(new PIDFCoefficients(0.01, 0, 0, 0));
        PanelsConfigurables.INSTANCE.refreshClass(this);
        servo.getController().setServoPwmEnable(0);
    }

    @Override
    public void run() {
        servo.setPIDF(new PIDFCoefficients(kP, 0, kD, 0));
        servo.set(setpointDegrees);
        updateTelemetry();
    }

    public void setTarget(double setpoint) {
        setpointDegrees = setpoint;
    }

    @Override
    protected void updateTelemetry(){
        telemetry.addLine("--------------SPINDEXER--------------");
        telemetry.addData("Position", servo.getAbsoluteEncoder().getCurrentPosition());
        telemetry.addData("Voltage", servo.getAbsoluteEncoder().getVoltage());
//        telemetry.addData("PID Error", );
    }

    @Override
    public void stop() {

    }
}
