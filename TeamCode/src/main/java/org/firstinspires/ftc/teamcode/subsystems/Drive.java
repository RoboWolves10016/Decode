package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.ProfiledPIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedropathing.Tuning;
import org.firstinspires.ftc.teamcode.util.PoseUtils;

import lombok.Getter;
import lombok.Setter;

@Configurable
public class Drive extends Subsystem{
    private final TelemetryManager telemetry;

    @Getter
    public Follower follower;

    private RobotState robotState;

    private boolean teleop = false;
    @Setter
    private boolean robotCentric = false;
    @Setter
    private boolean autoAim = false;
    private boolean lastAutoAim = false;

    private double autoAimTarget = 0;

    private final GamepadEx driver;

    private double forwardCommand = 0;
    private double strafeCommand = 0;
    private double turnCommand = 0;
    private double headingToGoal = 0;

    public static double aimP = 1;
    public static double aimI = 0;
    public static double aimD = 0.1;

//    ProfiledPIDController autoAimController = new ProfiledPIDController(
//            aimP,
//            aimI,
//            aimD,
//            new TrapezoidProfile.Constraints(0.5, 2));

    public double speedMultiplier = 1.0;

    private final PIDController rotationController = new PIDController(aimP,aimI,aimD);

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

//        rotationController.setPID(aimP, aimI, aimD);
        rotationController.setPID(aimP, aimI, aimD);

        forwardCommand = driver.getLeftY();// * speedMultiplier;
        strafeCommand = -driver.getLeftX();// * speedMultiplier;
        headingToGoal = MathFunctions.normalizeAngle(robotState.getVectorToGoal().getTheta());

        robotState.setNotMoving(
                follower.getAngularVelocity() < 1
                        && follower.getTeleopDriveVector().getMagnitude() < 1);

        // Accept vision pose if it is valid
        Pose visionPose = robotState.getVisionPose();
        if (visionPose != null && robotState.isLimelightEnabled() &&/* robotState.isNotMoving() &&*/ !autoAim) {
            follower.setPose(visionPose);
        }

        robotState.setPose(follower.getPose());

        if (teleop) {
            turnCommand = -driver.getRightX() * 0.75;
            if (autoAim) {
                if (!lastAutoAim) {
                    double heading = MathFunctions.normalizeAngle(follower.getHeading());
                    autoAimTarget = headingToGoal;
                    if (heading - autoAimTarget > Math.PI) {
                        autoAimTarget+= 2 * Math.PI;
                    }
                    if (autoAimTarget - heading > Math.PI) {
                        autoAimTarget -= 2 * Math.PI;
                    }
                }
//                turnCommand = autoAimController.calculate(
//                        PoseUtils.normalizeHeading(follower.getHeading()), autoAimTarget);
                turnCommand = rotationController.calculate(
                        MathFunctions.normalizeAngle(follower.getHeading()), autoAimTarget);
                if (turnCommand > 0.5) turnCommand = 0.5;
                if (turnCommand < -0.5) turnCommand = -0.5;
            }

//            turnCommand *= speedMultiplier;
            follower.setTeleOpDrive(
                    forwardCommand,
                    strafeCommand,
                    turnCommand,
                    robotCentric,
                    robotCentric ? 0 : robotState.getAlliance().driverForwardHeading
            );
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
            telemetry.addData("HeadingToGoal", headingToGoal);
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

    public void setSpeed(double pct) {
        speedMultiplier = pct;
    }
}