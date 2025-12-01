package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedropathing.Tuning;

import java.util.function.Supplier;

import lombok.Getter;
import lombok.Setter;

@Configurable
public class Drive extends Subsystem{
    private final TelemetryManager telemetry;

    @Getter
    public Follower follower;

    private RobotState robotState;

    private boolean teleop = false;
    private boolean robotCentric = false;

    private final GamepadEx driver;

    private double forwardCommand = 0;
    private double strafeCommand = 0;
    private double turnCommand = 0;
    private double headingToGoal = 0;

    private Supplier<PathChain> aimPath;
    private boolean slowMode = false;
    private boolean autoAim = false;
    private boolean lastAutoAim = false;

    public Drive(HardwareMap hwMap, GamepadEx driver) {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.follower = Constants.createFollower(hwMap);
        this.robotState = RobotState.getInstance();
        this.driver = driver;

        aimPath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, follower::getPose)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(robotState.getAlliance().goalPose))
                .build();
    }

    @Override
    public void init() {
        follower.setPose(robotState.getPose());
    }

    @Override
    public void run() {

        autoAim = driver.getButton(GamepadKeys.Button.LEFT_BUMPER);
        slowMode = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1;
        robotCentric = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1;

        headingToGoal = MathFunctions.normalizeAngle(robotState.getVectorToGoal().getTheta());

        forwardCommand = driver.getLeftY();
        strafeCommand = -driver.getLeftX();
        turnCommand = -driver.getRightX() * 0.5;

        robotState.setNotMoving(
                follower.getAngularVelocity() < 1
                        && follower.getTeleopDriveVector().getMagnitude() < 1);

        // Accept vision pose if it is valid
        Pose visionPose = robotState.getVisionPose();
        if (visionPose != null && robotState.isLimelightEnabled() && !autoAim) {
            follower.setPose(visionPose);
            if (robotState.isAuton()) {
                // Only use one pose at a time if in auton
                robotState.setLimelightEnabled(false);
            }
        }


        robotState.setPose(follower.getPose());

        if (teleop) {
            if (autoAim && !lastAutoAim) {
//                follower.turnTo(headingToGoal);
                follower.holdPoint(new BezierPoint(follower.getPose()), headingToGoal, false);
            } else if (!autoAim && lastAutoAim) {
                follower.startTeleopDrive(true);
            } else if (!slowMode && !autoAim) {
                // Not slow mode
                follower.setTeleOpDrive(
                        forwardCommand,
                        strafeCommand,
                        turnCommand,
                        robotCentric,
                        robotCentric ? 0 : robotState.getAlliance().driverForwardHeading);
            } else if (slowMode && !autoAim) {
                // Slow mode
                follower.setTeleOpDrive(
                        forwardCommand * 0.25,
                        strafeCommand * 0.25,
                        turnCommand * 0.5,
                        robotCentric,
                        robotCentric ? 0 : robotState.getAlliance().driverForwardHeading);
            }
        }
        lastAutoAim = autoAim;
        follower.update();
        updateTelemetry();
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------DRIVE--------------");
        if (teleop) {
            telemetry.addData("ForwardPower", forwardCommand);
            telemetry.addData("StrafePower", strafeCommand);
            telemetry.addData("TurnPower", turnCommand);
            telemetry.addData("HeadingToGoal", Math.toDegrees(headingToGoal));
        } else {
//            telemetry.addData("",);
        }
        Tuning.Drawing.drawDebug(follower);
    }

    @Override
    void stop() {

    }

    public void startTeleop() {
        teleop = true;

        follower.startTeleopDrive();
    }

    public void startAuton() {
        teleop = false;
    }
}