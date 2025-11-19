package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class GoalAutonPoses {

    public static Pose startPose;
    public static Pose launchPose;
    public static Pose row3Control;
    public static Pose postIntake3;
    public static Pose row2Control;
    public static Pose postIntake2;
    public static Pose row1Control;
    public static Pose postIntake1;
    public static Pose endPose;


    public static void setAlliance(Alliance alliance) {
        startPose = new Pose(127.5, 124.5, Math.toRadians(35.3));
        launchPose = new Pose(96, 96, Math.toRadians(50));
        row3Control = new Pose(86, 80);
        postIntake3 = new Pose(120, 84, Math.toRadians(0));
        row2Control = new Pose(76, 56);
        postIntake2 = new Pose(120, 60, Math.toRadians(0));
        row1Control = new Pose(75, 30);
        postIntake1 = new Pose(120, 36, Math.toRadians(0));
        endPose = new Pose(100, 72, Math.toRadians(0));

        if (alliance == Alliance.BLUE) {
            startPose = startPose.mirror();
            launchPose = launchPose.mirror();

            row3Control = row3Control.mirror();
            postIntake3 = postIntake3.mirror();

            row2Control = row2Control.mirror();
            postIntake2 = postIntake2.mirror();

            row1Control = row1Control.mirror();
            postIntake1 = postIntake1.mirror();

            endPose = endPose.mirror();
        }
    }
}
