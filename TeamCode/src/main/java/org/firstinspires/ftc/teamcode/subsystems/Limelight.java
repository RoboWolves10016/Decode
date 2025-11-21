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

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.util.Pattern;
import org.firstinspires.ftc.teamcode.util.PoseUtils;

import java.util.Locale;

public class Limelight extends Subsystem {

    private final HardwareMap hwMap;
    private final TelemetryManager telemetry;
    private final RobotState robotState;

    private final ElapsedTime timeSinceLastPose;

    private Limelight3A limelight;
    private Pose pose = new Pose(0,0,0);

    private boolean valid = false;
    private LLResult result;
    private LLStatus status;

    public Limelight(HardwareMap hwMap) {
        this.hwMap = hwMap;
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.timeSinceLastPose = new ElapsedTime();
        this.robotState = RobotState.getInstance();
    }

    @Override
    public void init() {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        timeSinceLastPose.reset();
    }

    @Override
    public void run() {
        // Publish basic telemetry status info
        status = limelight.getStatus();
//        telemetry.addData("Name", String.format("%s", status.getName()));
//        telemetry.addData("LL", String.format(Locale.US,"Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                status.getTemp(), status.getCpu(), (int) status.getFps()));
//        telemetry.addData("Pipeline", String.format(Locale.US, "Index: %d, Type: %s",
//                status.getPipelineIndex(), status.getPipelineType()));

        limelight.updateRobotOrientation(Math.toDegrees(robotState.getPose().getHeading()) + 90);

        result = limelight.getLatestResult();

        // If a valid result, update variables
        if (result.isValid()) {
            if (result.getBotpose() != null) {
                if (robotState.isHeadingInitialized()) {
//                    pose = PoseUtils.fromPose3d(result.getBotpose_MT2());
                } else {
                    pose = PoseUtils.fromPose3d(result.getBotpose());
                }
            }

            // Set obelisk pattern if visible
            for (LLResultTypes.FiducialResult r : result.getFiducialResults()) {
                if (r.getFiducialId() >= 21 && r.getFiducialId() <= 23) {
                    int obeliskID = r.getFiducialId();
                    if (obeliskID == 21) {
                        robotState.setPattern(Pattern.GPP);
                    } else if (obeliskID == 22){
                        robotState.setPattern(Pattern.PGP);
                    } else {
                        robotState.setPattern(Pattern.PPG);
                    }

                }
            }
        }

        valid = checkValidity(result);
        if (valid) timeSinceLastPose.reset();
        updateTelemetry();

        // Update state variables so other subsystems can access data
        robotState.setVisionPose(valid ? pose : null);
    }

    @Override
    public void updateTelemetry() {
        telemetry.addLine("--------------LIMELIGHT--------------");
        telemetry.addData("Pose", PoseUtils.poseToString(pose));
        telemetry.addData("Is Pose Valid", valid);
        telemetry.addData("Time Since Last Pose", timeSinceLastPose.seconds());
        telemetry.addData("FPS", status.getFps());
        telemetry.addData("PipelineType", status.getPipelineType());
    }

    private boolean checkValidity(LLResult result) {
        boolean tagIdCheck = false;
        for (LLResultTypes.FiducialResult r : result.getFiducialResults()) {
            if (r.getFiducialId() == 20 || r.getFiducialId() == 24) tagIdCheck = true;
        }
        return result.isValid()
                && result.getBotposeAvgArea() > 0.25
                && tagIdCheck && PoseUtils.isInField(pose)
                && robotState.isNotMoving();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}