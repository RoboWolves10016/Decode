package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

@Autonomous(name="Close 18")
public class Close18 extends AutonBase {
    private CloseAutonPaths paths;
    enum Cycle {
        ROW_1(0),
        ROW_2(0),
        ROW_3(0),
        GATE(0);
        final int leaveShootState;
        public PathChain firstPath;

        Cycle(int firstState) {
            this.leaveShootState = firstState;
        }
    }

    List<Cycle> cycleOrder;


//    private final double shotRpm = 3200;
//    private final double shotAngle = 47;
    private final double shotRpm = 3000;
    private final double shotAngle = 41;
    private final int gateCycles = 2;
    private final double gateTime = 1.5;
    private final double launchTime = 0.5;

    private int gateCycleCount = 0;
    private int cycleIndex = 0;

    @Override
    public void init() {
        cycleOrder = new ArrayList<>();
        cycleOrder.add(Cycle.ROW_2);
        super.init();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Configure your Auton!!!");
        telemetry.addLine("X: GATE\nA: Row 1\nB: Row 2\nY: Row 3\nBACK: Remove");
        telemetry.addLine("Actions:");
        telemetry.addLine("PRELOAD");
        cycleOrder.forEach(c -> telemetry.addLine(c.toString()));
        if (gamepad1.backWasPressed()) cycleOrder.remove(cycleOrder.size() - 1);
        if (gamepad1.xWasPressed()) cycleOrder.add(Cycle.GATE);
        if (gamepad1.aWasPressed()) cycleOrder.add(Cycle.ROW_1);
        if (gamepad1.bWasPressed()) cycleOrder.add(Cycle.ROW_2);
        if (gamepad1.yWasPressed()) cycleOrder.add(Cycle.ROW_3);
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        paths = new CloseAutonPaths(robotState.getAlliance(), follower);
        Cycle.ROW_1.firstPath = paths.launchToRow1;
        Cycle.ROW_2.firstPath = paths.launchToRow2;
        Cycle.ROW_3.firstPath = paths.launchToRow3;
        Cycle.GATE.firstPath = paths.launchToGateFull;
        follower.setStartingPose(paths.startPose);
        follower.setMaxPower(1.0);

        Flywheel.useManualRpm = true;
        Flywheel.manualRpm = shotRpm;

        Hood.useManualOverride = true;
        Hood.manualOverrideDeg = shotAngle;
    }

    @Override
    public void loop() {
        switch (autonState) {
            case 0: // Begin
                spinUp();
                follower.followPath(paths.startToLaunch);
                advanceState();
                break;
            case 1: // Drive to launch
//                if (!follower.isBusy()) {
                if (robotState.getVelocity().getMagnitude() < 5 && stateTimer.seconds() > 1) {
                    forceLaunch();
                    advanceState();
                }
                break;
            case 2: // Launch preloads
                if (stateTimer.seconds() > 0.5) {
                    stopForceLaunch();
                    intake();
                    follower.followPath(paths.launchToRow2);
                    advanceState();
                }
                break;
            case 3: // Intake row 2
                if (!follower.isBusy()) {
                    follower.followPath(paths.row2ToLaunch);
                    advanceState();
                }
                break;
            case 4: // Drive to launch
                if (!follower.isBusy()) {
                    forceLaunch();
                    advanceState();
                }
                break;
            case 5: // Launch row 2
                if (stateTimer.seconds() > launchTime) {
                    stopForceLaunch();
                    intake();
                    advanceState(10);
                }
                break;
            case 6: // Drive to gate
                if (!follower.isBusy() || stateTimer.seconds() > 1.5) {
//                    follower.followPath(paths.gate1ToGate2);
                    stopIntakeGate();
                    advanceState();
                }
                break;
            case 7: // Wait at gate
                if (stateTimer.seconds() > gateTime || robotState.isFull()) {
                    follower.followPath(paths.gate2ToLaunch);
                    advanceState();
                }
                break;
            case 8: // Drive to launch
                if (!follower.isBusy()) {
                    forceLaunch();
                    advanceState();
                }
                break;
            case 9: // Launch gate balls
                if (stateTimer.seconds() > launchTime) {
                    stopForceLaunch();
                    intake();
                    ++gateCycleCount;
                    advanceState();
                }
                break;
            case 10: // Decide where to go
                if (gateCycleCount < gateCycles) {
                    // Go back to gate
//                    follower.followPath(paths.launchToGate1);
//                    follower.setMaxPower(0.6);
                    follower.followPath(paths.launchToGateFull);
                    intakeGate();
                    advanceState(6);
                } else {
                    // Continue with rows
                    follower.followPath(paths.launchToRow1);
                    intake();
//                    Turret.useManualOverride = false;
                    advanceState();
                }
                break;
            case 11: // Intake row 1
                if (!follower.isBusy() || stateTimer.seconds() > 1.75) {
                    follower.followPath(paths.row1ToLaunch);
                    advanceState();
                }
                break;
            case 12: // Drive to launch
                if (!follower.isBusy()) {
                    forceLaunch();
                    advanceState();
                }
                break;
            case 13: // Launch
                if (stateTimer.seconds() > launchTime) {
                    stopForceLaunch();
                    intake();
                    follower.followPath(paths.launchToRow3);
                    advanceState();
                }
                break;
            case 14: // Drive to row 3
                if (!follower.isBusy()) {
                    follower.followPath(paths.row3ToLaunch);
                    advanceState();
                }
                break;
            case 15: // Drive to launch
                if (!follower.isBusy()) {
                    forceLaunch();
                    advanceState();
                }
                break;
            case 16: // Final launch
                if (stateTimer.seconds() > launchTime) {
                    stopForceLaunch();
                    spinDown();
                    intake();
                    follower.followPath(paths.launchToEnd);
                    advanceState();
                }
                break;
            case 17: // Drive to end
                if (!follower.isBusy()) {
                    advanceState();
                }
                break;
            case 18: // End; do nothing
                stop();
                break;
        }
        updateTelemetry();
        super.loop();
    }

    private void advanceCycle() {
        if (cycleIndex < cycleOrder.size()) autonState = cycleOrder.get(cycleIndex).leaveShootState;
        else autonState = 17;
        follower.followPath(cycleOrder.get(cycleIndex).firstPath);
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
