package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Tuning.AIR_TIMES;
import static org.firstinspires.ftc.teamcode.Tuning.DISTANCES_FROM_GOAL_INCHES;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedropathing.Tuning;
import org.firstinspires.ftc.teamcode.util.Interpolation;

import lombok.Getter;

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

    private boolean slowMode = false;
    private boolean autoAim = false;
    private boolean shootWhileMoving = false;
    private boolean lastAutoAim = false;

    public static double kP = 1.0;
    public static double kI = 0;
    public static double kD = 0.06;
    public static double kF = 0.02;
    private PIDFController aimController = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));

    public Drive(HardwareMap hwMap, GamepadEx driver) {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.follower = Constants.createFollower(hwMap);
        this.robotState = RobotState.getInstance();
        this.driver = driver;
    }

    @Override
    public void init() {
        follower.setPose(robotState.getPose());
    }

    @Override
    public void run() {
        aimController.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));

        autoAim = driver.getButton(GamepadKeys.Button.LEFT_BUMPER);
        slowMode = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1;
        robotCentric = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1;
        shootWhileMoving = driver.getButton(GamepadKeys.Button.RIGHT_BUMPER);

        headingToGoal = MathFunctions.normalizeAngle(robotState.getVectorToGoal().getTheta());

        forwardCommand = driver.getLeftY();
        strafeCommand = -driver.getLeftX();
        turnCommand = -driver.getRightX() * 0.85;

        robotState.setNotMoving(
                follower.getAngularVelocity() < 1
                        && follower.getTeleopDriveVector().getMagnitude() < 1);

        robotState.setAngularVelocity(follower.getAngularVelocity());

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
        Vector fieldVelocity = follower.getPoseTracker().getLocalizer().getVelocityVector();
        fieldVelocity.rotateVector(follower.getHeading());

        double timeGain = Interpolation.interpolate(
                DISTANCES_FROM_GOAL_INCHES,
                AIR_TIMES,
                robotState.getVectorToGoal().getMagnitude());
        robotState.setFuturePose(robotState.getPose().plus(new Pose(
                        fieldVelocity.getXComponent() * timeGain,
                        fieldVelocity.getYComponent() * timeGain)));

        if (teleop) {
            if (shootWhileMoving) {

                double heading = MathFunctions.normalizeAngle(follower.getHeading() + Math.PI);
                double autoAimTarget = headingToGoal;
                if (heading - autoAimTarget > Math.PI) {
                    autoAimTarget+= 2 * Math.PI;
                }
                if (autoAimTarget - heading > Math.PI) {
                    autoAimTarget -= 2 * Math.PI;
                }
                aimController.updatePosition(heading);
                aimController.setTargetPosition(autoAimTarget);
    
                follower.setTeleOpDrive(
                        MathUtils.clamp(forwardCommand * 0.5, -0.35, 0.35),
                        MathUtils.clamp(strafeCommand * 0.5, -0.35, 0.35),
                        aimController.run(),
                        robotCentric,
                        robotCentric ? 0 : robotState.getAlliance().driverForwardHeading);

            } else if (autoAim && !lastAutoAim) {
                follower.holdPoint(new BezierPoint(follower.getPose()), headingToGoal, false);
            }else if (!autoAim && lastAutoAim) {
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