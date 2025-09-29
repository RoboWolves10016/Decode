package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Pattern;
import org.firstinspires.ftc.teamcode.util.PoseUtils;

import java.util.Locale;

public class Limelight implements ISubsystem {

    private final HardwareMap hwMap;
    private final TelemetryManager telemetry;

    private final ElapsedTime timeSinceLastPose;

    private Limelight3A limelight;
    private Pose pose = new Pose(0,0,0);

    private static int obeliskID;

    private boolean valid;

    public Limelight(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.timeSinceLastPose = new ElapsedTime();
        telemetry.setMsTransmissionInterval(11);
    }

    @Override
    public void init() {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        timeSinceLastPose.reset();
    }

    @Override
    public void periodic() {
        // Publish basic telemetry status info
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", String.format("%s", status.getName()));
        telemetry.addData("LL", String.format(Locale.US,"Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps()));
        telemetry.addData("Pipeline", String.format(Locale.US, "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType()));

        LLResult result = limelight.getLatestResult();

        // If a valid result, update variables
        if (result.isValid()) {
            if (result.getBotpose() != null) {
                pose = PoseUtils.fromPose3d(result.getBotpose());
            }

            // Set obelisk AprilTag ID if visible
            for (LLResultTypes.FiducialResult r : result.getFiducialResults()) {
                if (r.getFiducialId() >= 21 && r.getFiducialId() <= 23) {
                    obeliskID = r.getFiducialId();
                }
            }
        }

        valid = checkValidity(result);
        if (valid) timeSinceLastPose.reset();
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addLine("--------------LIMELIGHT--------------");
        telemetry.addData("Pose", PoseUtils.poseToString(pose));
        telemetry.addData("Pattern", getPattern());
        telemetry.addData("Is Pose Valid", valid);
        telemetry.addData("Time Since Last Pose", timeSinceLastPose.seconds());
    }

    public Pose getPose() {
        return pose;
    }

    public Pattern getPattern() {
        if (obeliskID == 21) {
            return Pattern.GPP;
        } else if (obeliskID == 22){
            return Pattern.PGP;
        } else {
            return Pattern.PPG;
        }
    }

    private boolean checkValidity(LLResult result) {
        boolean tagIdCheck = false;
        for (LLResultTypes.FiducialResult r : result.getFiducialResults()) {
            if (r.getFiducialId() == 20 || r.getFiducialId() == 24) tagIdCheck = true;
        }
        return result.isValid()
                && result.getBotposeAvgArea() > 1
                && tagIdCheck && PoseUtils.isInField(pose);
    }

    public boolean isValid() {
        return false;
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}