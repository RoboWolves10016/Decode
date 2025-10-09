package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name = "Red Loading Zone")
public class RedLoadingZone extends OpMode {

    private Follower follower;

    private int autonState = 1;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        switch(autonState) {
            case 1:
                // Code to make drivetrain move from start to launch zone
                follower.followPath(follower.pathBuilder()
                        .addPath(
                                // StartToShoot
                                new BezierCurve(
                                        new Pose(41.502, 8.384),
                                        new Pose(41.502, 26.830),
                                        new Pose(72.943, 71.686),
                                        new Pose(53.031, 90.131)
                                )
                        ).setTangentHeadingInterpolation().build());
                if (!follower.isBusy()) {
                    autonState = autonState + 1;
                }
                break;
            case 2:
                // Code to trigger launcher and spindexer scoring sequence
                break;

            case 3:
                break;

        }
        follower.update();
    }
}
