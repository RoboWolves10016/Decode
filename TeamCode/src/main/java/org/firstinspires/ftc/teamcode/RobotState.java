package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.PoseUtils.poseToString;
import static org.firstinspires.ftc.teamcode.util.PoseUtils.vectorToString;

import androidx.annotation.Nullable;

import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Pattern;

import lombok.Getter;
import lombok.Setter;

@Getter
public class RobotState {

    private static RobotState instance;

//    private final PanelsField field = PanelsField.INSTANCE;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
    private RobotState() {}

    /*** BEGIN ROBOT STATE VARIABLES ***/
    @Setter
    private Alliance alliance = Alliance.RED;

    @Setter
    private Pattern pattern = Pattern.GPP;

    private Pose pose = new Pose();


    public void setPose(Pose pose) {
        this.pose = pose;
        this.vectorToGoal = alliance.goalPose.minus(pose).getAsVector();
        this.vectorToGoal.rotateVector(-pose.getHeading());
    }

    private Vector vectorToGoal = alliance.goalPose.minus(pose).getAsVector();;
    {
        vectorToGoal.rotateVector(-pose.getHeading());
    }

    @Setter @Nullable
    private Pose visionPose = null;

    public void addTelemetry(TelemetryManager telemetry) {
        telemetry.addLine("--------------ROBOT STATE--------------");
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Pattern", pattern.toString());
        telemetry.addData("Pose", poseToString(pose));
        telemetry.addData("Vision Pose", visionPose == null ? "None" : poseToString(visionPose));
        telemetry.addData("Vector to Goal", vectorToGoal == null ? "None" : vectorToString(vectorToGoal));
    }
}
