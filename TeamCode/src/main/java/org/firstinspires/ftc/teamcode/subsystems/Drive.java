package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;

import lombok.Setter;

public class Drive extends Subsystem{
    private final TelemetryManager telemetry;
    public Follower follower;

    private RobotState robotState;

    private boolean teleop = false;
    @Setter
    private boolean autoAim = false;

    private final GamepadEx driver;

    private double forwardCommand = 0;
    private double strafeCommand = 0;
    private double turnCommand = 0;
    private double headingToGoal = 0;

    private final PIDController rotationController = new PIDController(2,0,0.1);

    public Drive(HardwareMap hwMap, GamepadEx driver) {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.follower = Constants.createFollower(hwMap);
        this.robotState = RobotState.getInstance();
        this.driver = driver;
    }

    @Override
    public void init() {
        follower.setStartingPose(
                robotState.getPose() == null ? new Pose() : robotState.getPose()
        );
    }

    @Override
    public void run() {

        forwardCommand = driver.getLeftY();
        strafeCommand = -driver.getLeftX();
        headingToGoal = robotState.getVectorToGoal().getTheta();

        robotState.setNotMoving(
                follower.getVelocity().getMagnitude() < 1
                        && follower.getHeadingVector().getMagnitude() < 1);
        // Accept vision pose if it is valid
        Pose visionPose = robotState.getVisionPose();
        if (visionPose != null) {
            follower.setPose(visionPose);
            robotState.setPose(visionPose);
        } else {
            robotState.setPose(follower.getPose());
        }

        if (teleop) {
            turnCommand = -driver.getRightX();
            if (autoAim) {
                turnCommand = rotationController.calculate(
                        follower.getHeading(),
                        headingToGoal
                );
            }

            follower.setTeleOpDrive(
                    forwardCommand,
                    strafeCommand,
                    turnCommand,
                    false,
                    robotState.getAlliance().driverForwardHeading
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
}