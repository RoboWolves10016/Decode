package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

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
    public Follower follower;
    private RobotState robotState;

    private boolean teleop = false;
    @Setter
    private boolean autoAim = false;

    private final GamepadEx driver;

    private final PIDController rotationController = new PIDController(2,0,0.1);

    public Drive(HardwareMap hwMap, GamepadEx driver) {
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
        // Accept vision pose if it is valid
        Pose visionPose = robotState.getVisionPose();
        if (visionPose != null) {
            follower.setPose(visionPose);
            robotState.setPose(visionPose);
        } else {
            robotState.setPose(follower.getPose());
        }

        if (teleop) {
            double rotation = -driver.getRightX();
            if (autoAim) {
                rotation = rotationController.calculate(
                        follower.getHeading(),
                        robotState.getVectorToGoal().getTheta()
                );
            }

            follower.setTeleOpDrive(
                    -driver.getLeftY(),
                    -driver.getLeftX(),
                    rotation,
                    false,
                    robotState.getAlliance().driverForwardHeading
            );
        }
        follower.update();

    }

    @Override
    protected void updateTelemetry() {
    }

    @Override
    void stop() {

    }

    public void startTeleop() {
        teleop = true;

        follower.startTeleopDrive();
    }
}