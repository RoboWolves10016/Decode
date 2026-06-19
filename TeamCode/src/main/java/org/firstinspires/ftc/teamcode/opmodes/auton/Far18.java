package org.firstinspires.ftc.teamcode.opmodes.auton;

import android.app.admin.PolicyUpdateReceiver;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Far 18")
public class Far18 extends AutonBase {
    private FarAutonPaths paths;
    enum Cycle {
        ROW_2(10),
        ROW_3(20),
        PURE_CORNER(30),
        CORNER_AND_WALL(40),
        WALL(50),
        END(60);
        final int firstState;

        Cycle(int firstState) {
            this.firstState = firstState;
        }
    }

    List<Cycle> cycleOrder;


//    private final double shotRpm = 3200;
//    private final double shotAngle = 47;
    private final double shotRpm = 3900;
    private final double shotAngle = 48;
    private final double turretAngle = -111;
    private final double launchTime = 1.0;

    private int cycleIndex = 0;

    @Override
    public void init() {
        cycleOrder = new ArrayList<>();
        cycleOrder.add(Cycle.ROW_3);
        cycleOrder.add(Cycle.PURE_CORNER);
        super.init();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Configure your Auton!!!");
        telemetry.addLine("X: Corner1\nA: Wall Line\nB: Row 2\nY: Row 3\nBACK: Remove");
        telemetry.addLine("-----Current Actions-----");
        telemetry.addLine("PRELOAD");
        cycleOrder.forEach(c -> telemetry.addLine(c.toString()));
        if (gamepad2.backWasPressed()) cycleOrder.remove(cycleOrder.size() - 1);
        if (gamepad2.xWasPressed()) cycleOrder.add(Cycle.PURE_CORNER);
        if (gamepad2.aWasPressed()) cycleOrder.add(Cycle.WALL);
        if (gamepad2.bWasPressed()) cycleOrder.add(Cycle.ROW_2);
        if (gamepad2.yWasPressed()) cycleOrder.add(Cycle.ROW_3);
        if (gamepad2.startWasPressed()) cycleOrder.add(Cycle.CORNER_AND_WALL);
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        paths = new FarAutonPaths(robotState.getAlliance(), follower);
        follower.setStartingPose(paths.startPose);
        follower.setMaxPower(1.0);

        Flywheel.useManualRpm = true;
        Flywheel.manualRpm = shotRpm;

        Hood.useManualOverride = true;
        Hood.manualOverrideDeg = shotAngle;

        Turret.useManualOverride = true;
        Turret.manualOverrideDegrees = robotState.getAlliance() == Alliance.RED ? turretAngle : -turretAngle;
    }

    @Override
    public void loop() {
        switch (autonState) {
            case 0: // Begin
                spinUp();
                follower.followPath(paths.startToLaunch);
                advanceState();
                break;
            case 1: // Wait for spin up and drive
                if (!follower.isBusy() && stateTimer.seconds() > 2.0) {
                    advanceState();
                }
                break;
            case 2: // RETURN TO LAUNCH
                if (!follower.isBusy()) {
                    follower.holdPoint(paths.launchPose);
                    forceLaunch();
                    advanceState();
                }
                break;
            case 3: // LAUNCH + CYCLE ADVANCE
                if (stateTimer.seconds() > launchTime) {
                    stopForceLaunch();
                    intake();
                    advanceCycle();
                }
                break;

            case 10: // BEGIN ROW 2
                follower.followPath(paths.launchToRow2);
                advanceState();
                break;
            case 11: // INTAKE ROW 2
                if (!follower.isBusy() || robotState.isFull()) {
                    follower.followPath(paths.row2ToLaunch);
                    advanceState(2); // go to launch + advance
                }
                break;

            case 20: // BEGIN ROW 3
                follower.followPath(paths.launchToRow3);
                advanceState();
                break;
            case 21: // INTAKE ROW 3
                if (!follower.isBusy() || robotState.isFull()) {
                    follower.followPath(paths.row3ToLaunch);
                    advanceState(2); // go to launch + advance
                }
                break;

            case 30: // BEGIN PURE CORNER
                follower.followPath(paths.launchToPureCorner1);
                advanceState();
                break;
            case 31: // INTAKE PURE CORNER
                if (!follower.isBusy() || robotState.isFull() || stateTimer.seconds() > 2.5) {
                    follower.followPath(paths.launchToPureCorner1);
                    advanceState(); // go to launch + advance
                }
                break;
            case 32: // FINISH INTAKE, RETURN
                if (!follower.isBusy() || robotState.isFull() || stateTimer.seconds() > 2.5) {
                    follower.followPath(paths.pureCornerToLaunch);
                    advanceCycle();
                }
                break;
            case 40: // BEGIN CORNER AND WALL
                follower.followPath(paths.launchToCornerAndWall);
                advanceState();
                break;
            case 41:
                if (!follower.isBusy() || robotState.isFull()) {
                    follower.followPath(paths.wallToLaunch);
                    advanceState(2); // go to launch + advance
                }
                break;

            case 50: // BEGIN WALL ONLY
                follower.followPath(paths.launchToWall);
                advanceState();
                break;
            case 51:
                if (!follower.isBusy() || robotState.isFull()) {
                    follower.followPath(paths.wallToLaunch);
                    advanceState(2); // go to launch + advance
                }
                break;

            case 60: // BEGIN END
                follower.followPath(paths.launchToEnd);
                break;
            case 61:
                if (!follower.isBusy()) {
                    stop();
                }
                break;
        }
        updateTelemetry();
        super.loop();
    }

    private void advanceCycle() {
        if (autonTimer.seconds() < 25 && cycleIndex < cycleOrder.size()) {
            autonState = cycleOrder.get(cycleIndex).firstState;
        } else {
            autonState = Cycle.END.firstState;
        }
        ++cycleIndex;
    }

    private void updateTelemetry() {
        telemetry.addLine("---------PEDRO AUTON---------");
        telemetry.addData("State Duration", stateTimer.seconds());
        telemetry.addData("Auton State", autonState);
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.addData("Follower Pose", follower.getPose());
        telemetry.addData("T-Value", follower.getCurrentTValue());
        telemetry.addData("Path Number", follower.getCurrentPathNumber());
    }


}
