package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.ProfiledPIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
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

    private final GamepadEx driver;

    private double forwardCommand = 0;
    private double strafeCommand = 0;
    private double turnCommand = 0;
    private double headingToGoal = 0;

    public static double aimP = 1;
    public static double aimI = 0;
    public static double aimD = 0.1;

    ProfiledPIDController autoAimController = new ProfiledPIDController(
            aimP,
            aimI,
            aimD,
            new TrapezoidProfile.Constraints(0.5, 2));

    public double speedMultiplier = 1.0;

//    private final PIDController rotationController = new PIDController(aimP,aimI,aimD);

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
        autoAimController.setPID(aimP, aimI, aimD);

        forwardCommand = driver.getLeftY();// * speedMultiplier;
        strafeCommand = -driver.getLeftX();// * speedMultiplier;
        headingToGoal = PoseUtils.normalizeHeading(robotState.getVectorToGoal().getTheta());

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
            turnCommand = -driver.getRightX() * 0.5;
            if (autoAim) {
                double heading = PoseUtils.normalizeHeading(follower.getHeading());
                double targetHeading = headingToGoal;
                if (heading - targetHeading > Math.PI) {
                    targetHeading += 2 * Math.PI;
                }
                if (targetHeading - heading > Math.PI) {
                    heading += 2 * Math.PI;
                }
                turnCommand = autoAimController.calculate(heading, targetHeading);
//                turnCommand = rotationController.calculate(
//                        heading,
//                        targetHeading);
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
        follower.update();
        updateTelemetry();
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------DRIVE--------------");
        telemetry.addData("ForwardPower", forwardCommand);
        telemetry.addData("StrafePower", strafeCommand);
        telemetry.addData("TurnPower", turnCommand);
        telemetry.addData("HeadingToGoal", headingToGoal);
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