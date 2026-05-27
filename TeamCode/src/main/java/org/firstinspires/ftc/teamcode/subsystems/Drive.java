package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Tuning.AIR_TIMES;
import static org.firstinspires.ftc.teamcode.Tuning.DISTANCES_FROM_GOAL_INCHES;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedropathing.FusionLocalizer;
import org.firstinspires.ftc.teamcode.util.Interpolation;
import org.firstinspires.ftc.teamcode.util.VisionPose;

import lombok.Getter;

@Configurable
public class Drive extends Subsystem{
    private final TelemetryManager telemetry;

    @Getter
    private Follower follower;

    private final RobotState robotState;

    private final FusionLocalizer fusion;
    public static double xyVarianceIn2 = 0;
    public static double thetaVarianceRad2 = 0;

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
        this.fusion = new FusionLocalizer(
                new PinpointLocalizer(hwMap, Constants.pinpointConstants),
                new Pose(0.25, 0.25, Math.toRadians(2)),
                new Pose(1, 1, Math.toRadians(0.5) / 60),
                new Pose(2.1561, 2.6065, 0.0248),
                100
        );
        this.follower = new FollowerBuilder(Constants.followerConstants, hwMap)
                .pathConstraints(Constants.pathConstraints)
                .mecanumDrivetrain(Constants.mecanumConstants)
//                .setLocalizer(fusion)
                .pinpointLocalizer(Constants.pinpointConstants)
                .build();
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
        VisionPose visionPose = robotState.getVisionPose();
        if (visionPose != null && robotState.isLimelightEnabled() && !autoAim) {
            follower.setPose(visionPose.pose);
//            fusion.addMeasurement(
//                    visionPose.pose,
//                    visionPose.timestampNs,
//                    new Pose(xyVarianceIn2, xyVarianceIn2, thetaVarianceRad2));
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
//        Tuning.Drawing.drawDebug(follower);
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