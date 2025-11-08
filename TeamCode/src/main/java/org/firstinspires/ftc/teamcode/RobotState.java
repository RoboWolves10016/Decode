package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.PoseUtils.poseToString;
import static org.firstinspires.ftc.teamcode.util.PoseUtils.vectorToString;

import androidx.annotation.Nullable;

import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Pattern;
import org.firstinspires.ftc.teamcode.util.SpindexerSlot;

import java.util.HashMap;
import java.util.Map;

import lombok.Getter;
import lombok.Setter;

@Getter
public class RobotState {

    private static RobotState instance;


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

    private Pose pose;
    {
        setPose(new Pose());
    }

    @Setter
    private boolean kickerSafe = true;

    @Setter
    private boolean spindexerAlignedForLaunch = false;

    @Setter
    private boolean spindexerAlignedForIntake = false;

    public void setPose(Pose pose) {
        this.pose = pose;
        this.vectorToGoal = alliance.goalPose.minus(pose).getAsVector();
    }

    private Vector vectorToGoal;

    @Setter @Nullable
    private Pose visionPose = null;

    @Setter
    private boolean notMoving = true;

    @Setter
    SpindexerSlot currentSlot = SpindexerSlot.ONE;

    @Getter @Setter
    private boolean ballKicked = false;

    @Getter @Setter
    private boolean rpmReady = false;

    @Getter @Setter
    private boolean isFull = false;

    public void addTelemetry(TelemetryManager telemetry) {
        telemetry.addLine("--------------ROBOT STATE--------------");
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Pattern", pattern.toString());
        telemetry.addData("Pose", poseToString(pose));
        telemetry.addData("Goal Pose", poseToString(alliance.goalPose));
        telemetry.addData("Vision Pose", visionPose == null ? "None" : poseToString(visionPose));
        telemetry.addData("Vector to Goal", vectorToGoal == null ? "None" : vectorToString(vectorToGoal));
        telemetry.addData("Is Kicker Safe?", kickerSafe);
        telemetry.addData("SpindexerAlignedLaunch", spindexerAlignedForLaunch);
        telemetry.addData("SpindexerAlignedIntake", spindexerAlignedForIntake);

    }
}
