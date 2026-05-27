package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.PoseUtils.poseToString;
import static org.firstinspires.ftc.teamcode.util.PoseUtils.vectorToString;

import androidx.annotation.Nullable;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Pattern;
import org.firstinspires.ftc.teamcode.util.SpindexerSlot;
import org.firstinspires.ftc.teamcode.util.VisionPose;

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
    private double angularVelocity = 0;
//    @Setter
//    private double angularAcceleration = 0;

    private Pose futurePose;
    {
        setFuturePose(new Pose());
    }

    public void setPose(Pose pose) {
        this.pose = pose;
        this.vectorToGoal = alliance.goalPose.minus(pose).getAsVector();
    }

    public void setFuturePose(Pose pose) {
        this.futurePose = pose;
        this.futureVectorToGoal = alliance.goalPose.minus(futurePose).getAsVector();
    }

    private Vector vectorToGoal;
    private Vector futureVectorToGoal;

    @Setter @Nullable
    private VisionPose visionPose = null;

    @Setter
    private boolean notMoving = true;


    @Setter
    private boolean launcherReady = false;


    @Setter
    private boolean isIndexerLoaded = false;
    @Setter
    private boolean isIntakeFull = false;
    @Setter
    private boolean has3Balls = false;
    @Setter
    private boolean isFull = false;

    @Setter
    boolean headingInitialized = false;

    @Setter
    boolean limelightEnabled = true;

    @Setter
    boolean auton = true;

    double lastTimeStamp = -1;


    public void addTelemetry(TelemetryManager telemetry) {
        telemetry.addLine("--------------ROBOT STATE--------------");
        double newTimestamp = System.currentTimeMillis();
        telemetry.addData("Last Loop Time (ms)", newTimestamp - lastTimeStamp);
        lastTimeStamp = newTimestamp;
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Pattern", pattern.toString());
        telemetry.addData("Pose", poseToString(pose));
        telemetry.addData("Future Pose", poseToString(futurePose));
        telemetry.addData("Goal Pose", poseToString(alliance.goalPose));
        telemetry.addData("Heading Initialized", headingInitialized);
        telemetry.addData("Use Limelight", limelightEnabled);
        telemetry.addData("Vision Pose", visionPose == null ? "None" : poseToString(visionPose.pose));
        telemetry.addData("Vector to Goal", vectorToGoal == null ? "None" : vectorToString(vectorToGoal));
        telemetry.addData("Distance to Goal", vectorToGoal == null ? "None" : vectorToGoal.getMagnitude());
        telemetry.addData("LauncherReady", launcherReady);
    }
}
