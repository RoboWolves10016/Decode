package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.Optional;

public class Limelight implements ISubsystem {
    private static final String SERIAL_NUMBER = "";
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private Optional<Pose> pose = Optional.empty();
    private int tagCount = 0;

    public Limelight(HardwareMap hwMap, Telemetry telemetry) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        telemetry.setMsTransmissionInterval(11);
    }

    @Override
    public void init() {
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());

            telemetry.addData("Botpose", botpose.toString());
            telemetry.update();
        }
    }

    private void updateTelemetry() {
//        String data = "";
//        if (!pose.isPresent()) {
//            data = "Optional.empty()";
//        } else {
//            data = "(" + pose.get().getX() + ", " + pose.get().getY() + ", " + pose.get().getHeading() + ")";
//        }
//        telemetry.addData(
//                "Limelight Pose (x, y, yaw)",
//                data);
        telemetry.addData("Tags Present: ", tagCount);
        telemetry.update();
    }

}