package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.GoalAutonPoses.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedropathing.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BallState;
import org.firstinspires.ftc.teamcode.util.SpindexerSlot;

@Autonomous(name = "Red Goal")
public class RedGoal extends OpMode {
    private final RobotState robotState = RobotState.getInstance();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private final Alliance alliance = Alliance.RED;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    // Paths
    private Drive drivetrain;
    private Follower follower;
    private Launcher launcher;
    private Spindexer spindexer;
    private Intake intake;
    private Limelight limelight;
    private Kicker kicker;
    private int autonState = 0;

    @Override
    public void init() {
        drivetrain = new Drive(hardwareMap, new GamepadEx(gamepad1));
        drivetrain.init();
        drivetrain.startAuton();
        follower = drivetrain.getFollower();
        follower.setStartingPose(new Pose(125.135, 121.153, Math.toRadians(35.3)));
        launcher = new Launcher(hardwareMap);
        launcher.init();
        spindexer = new Spindexer(hardwareMap);
        spindexer.init();
        kicker = new Kicker(hardwareMap);
        kicker.init();
        intake = new Intake(hardwareMap);
        intake.init();
        limelight = new Limelight(hardwareMap);
        limelight.init();

        robotState.setAlliance(alliance);
        robotState.setLimelightEnabled(false);
        GoalAutonPoses.setAlliance(alliance);
        GoalAutonPoses.createPaths(follower, alliance);

    }

    @Override
    public void init_loop() {
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        spindexer.setSlotData(SpindexerSlot.ONE, BallState.GREEN);
        spindexer.setSlotData(SpindexerSlot.TWO, BallState.PURPLE);
        spindexer.setSlotData(SpindexerSlot.THREE, BallState.PURPLE);
        spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
        spindexer.setFeedType(Spindexer.FeedType.PATTERN);
        spindexer.setLaunchMode();
        stateTimer.reset();
    }

    @Override
    public void loop() {
        updateTelemetry();
        RobotState.getInstance().addTelemetry(telemetryM);

        switch (autonState) {
            case 0:
                follower.followPath(startToLaunch);
                advanceAutonState();
                break;
            case 1:
                // Go to launch position
//                follower.followPath(startToLaunch);
                launcher.setAuto();
                if (!follower.isBusy() || stateTimer.seconds() > 2) {
                    advanceAutonState();
                }
                break;
            case 2:
                // Shoot
                intake.stopIntake();
                kicker.feed();
                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    follower.followPath(launchToRow3);
                }
                break;
            case 3:
                // Follow ball intake path and intake
                launcher.setIdle();
                intake.runIntake();
                kicker.resetHistory();
                spindexer.setIntakeMode();

                if (!follower.isBusy() ||  stateTimer.seconds() > 5) {
                    advanceAutonState();
                    follower.followPath(row3ToLaunch);
                    follower.setMaxPower(1.0);
                }
                break;
            case 4:
                if (stateTimer.seconds() > 1) advanceAutonState();
            case 5:
                // Go to launch position
                launcher.setAuto();

                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    advanceAutonState();
                    spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
                    spindexer.setFeedType(Spindexer.FeedType.PATTERN);
                    spindexer.setLaunchMode();
                }
                break;
            case 6:
                // Shoot
                intake.stopIntake();
                kicker.feed();

                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    kicker.stopFeed();
                    launcher.setIdle();
                    follower.followPath(launchToRow2);
                }
                break;
            case 7:
                // Start intaking and drive to row 2
                intake.runIntake();
                spindexer.setIntakeMode();
                kicker.resetHistory();

                if (!follower.isBusy() || stateTimer.seconds() > 7) {
                    advanceAutonState();
                    follower.setMaxPower(1.0);
                    follower.followPath(row2ToLaunch);
                }
                break;
            case 8:
                if (stateTimer.seconds() > 1) advanceAutonState();
            case 9:
                // Drive back to launch position
                launcher.setAuto();

                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    advanceAutonState();
                    spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
                    spindexer.setFeedType(Spindexer.FeedType.PATTERN);
                    spindexer.setLaunchMode();
                }
                break;

            case 10:
                // Shoot
                intake.stopIntake();
                kicker.feed();
                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    kicker.stopFeed();
                    launcher.setIdle();
                    follower.followPath(launchToRow1);
                }
                break;
            case 11:
                // Start intaking and drive to row 1
                intake.runIntake();
                kicker.resetHistory();
                spindexer.setIntakeMode();

                if (!follower.isBusy() || stateTimer.seconds() > 10) {
                    advanceAutonState();
                    follower.setMaxPower(1.0);
                    follower.followPath(row1ToLaunch);
                }
                break;
            case 12:
                if (stateTimer.seconds() > 1) advanceAutonState();
            case 13:
                // Drive back to launch position
                launcher.setAuto();
                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    advanceAutonState();
                    spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
                    spindexer.setFeedType(Spindexer.FeedType.PATTERN);
                    spindexer.setLaunchMode();
                }
                break;
            case 14:
                // Launch
                intake.stopIntake();
                kicker.feed();
                if (spindexer.isEmpty()) {
                    advanceAutonState();
                    kicker.stopFeed();
                    launcher.setIdle();
                    follower.followPath(launchToEnd);
                }
                break;
            case 15:
                // Move off line
                if (!follower.isBusy()) {
                    // Do Nothing
                }
                break;
        }

        Tuning.Drawing.drawDebug(follower);
        drivetrain.run();
        intake.run();
        spindexer.run();
        kicker.run();
        launcher.run();
        limelight.run();
        telemetryM.update(telemetry);
    }

    private void updateTelemetry() {
        telemetryM.addLine("---------PEDRO AUTON---------");
        telemetryM.addData("State Duration", stateTimer.seconds());
        telemetryM.addData("Auton State", autonState);
        telemetryM.addData("Is Busy", follower.isBusy());
        telemetryM.addData("Follower Pose", follower.getPose());
        telemetryM.addData("T-Value", follower.getCurrentTValue());
        telemetryM.addData("Path Number", follower.getCurrentPathNumber());
        telemetryM.update(telemetry);
    }

    private void advanceAutonState() {
        autonState = autonState + 1;
        stateTimer.reset();
    }

    private void advanceAutonState(int newState) {
        autonState = newState;
        stateTimer.reset();
    }




    public void drawOnlyCurrent() {
        try {
            Tuning.Drawing.drawRobot(follower.getPose());
            Tuning.Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }


}
