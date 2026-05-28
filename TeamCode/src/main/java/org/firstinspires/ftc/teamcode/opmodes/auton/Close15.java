package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

@Autonomous(name="Close 15")
public class Close15 extends AutonBase {
    enum Cycle {
        ROW_1(0),
        ROW_2(0),
        ROW_3(0),
        GATE(0);
        final int leaveShootState;

        Cycle(int firstState) {
            this.leaveShootState = firstState;
        }
    }

    private CloseAutonPaths paths;

    private final double shotRpm = 31;
    private final double shotAngle = 46;
    private final int gateCycles = 2;
    private final double gateTime = 2.5;
    private final double launchTime = 0.65;

    private int gateCycleCount = 0;

    @Override
    public void start() {
        super.start();
        paths = new CloseAutonPaths(robotState.getAlliance(), follower);
        follower.setStartingPose(paths.startPose);
        follower.setMaxPower(1.0);

//        Flywheel.useManualRpm = true;
//        Flywheel.manualRpm = shotRpm;

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
                    follower.followPath(paths.launchToGate);
                    follower.setMaxPower(0.6);
                    advanceState();
                }
                break;
            case 6: // Drive to gate
                if (!follower.isBusy() || robotState.isFull()) {
                    advanceState();
                }
                break;
            case 7: // Wait at gate
                if (stateTimer.seconds() > gateTime || robotState.isFull()) {
                    follower.followPath(paths.gateToLaunch);
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
                    follower.followPath(paths.launchToGate);
                    advanceState(6);
                } else {
                    follower.followPath(paths.launchToRow1);
                    intake();
                    advanceState();
                }
                break;
            case 11: // Intake row 1
                if (!follower.isBusy()) {
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
            case 13: // Final launch
                if (stateTimer.seconds() > launchTime) {
                    stopForceLaunch();
                    spinDown();
                    follower.followPath(paths.launchToEnd);
                    advanceState();
                }
                break;
            case 14: // Drive to end
                if (!follower.isBusy()) {
                    advanceState();
                }
                break;
            case 15: // End; do nothing
                break;

        }
        super.loop();
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
