package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name="Far 18")
public class Far18 extends OpMode {
    private final RobotState robotState = RobotState.getInstance();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private final Alliance alliance = Alliance.BLUE;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    // Paths
    private Drive drivetrain;
    private Follower follower;
    private Launcher launcher;
    private Intake intake;
    private Indexer indexer;
    private Limelight limelight;
    private int autonState = 0;

    private int numCycles = 0;

    @Override
    public void init() {
        drivetrain = new Drive(hardwareMap, new GamepadEx(gamepad1));
        drivetrain.init();
        drivetrain.startAuton();
        follower = drivetrain.getFollower();

//        FarAutonPaths.setAlliance(alliance);
//        FarAutonPaths.createPaths(follower);

        launcher = new Launcher(hardwareMap);
        launcher.init();

        indexer = new Indexer(hardwareMap);
        indexer.init();

//        MotorEx indexer = new MotorEx(hardwareMap, "IndexerMotor")

        intake = new Intake(hardwareMap);
        intake.init();



//        limelight = new Limelight(hardwareMap);
//        limelight.init();

//        robotState.setAlliance(alliance);
        robotState.setLimelightEnabled(false);

    }

    @Override
    public void init_loop() {
//        limelight.run();
        follower.update();
        telemetryM.addData("Starting Pose", FarAutonPaths.startingPose);
        telemetry.addData("Alliance", robotState.getAlliance());
//        drawOnlyCurrent();
        if (gamepad1.b) RobotState.getInstance().setAlliance(Alliance.RED);
        if (gamepad1.x) RobotState.getInstance().setAlliance(Alliance.BLUE);



//        telemetryM.addData("Sees Pattern", limelight.isHasSeenPattern());
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        stateTimer.reset();
        FarAutonPaths.setAlliance(robotState.getAlliance());
        FarAutonPaths.createPaths(follower);
        follower.setStartingPose(FarAutonPaths.startingPose);
        follower.update(); // TRY REMOVING
    }

    @Override
    public void loop() {
        updateTelemetry();
        RobotState.getInstance().addTelemetry(telemetryM);

        switch (autonState) {
            case 0:
                follower.setMaxPower(0.75);
                launcher.setActive();
                follower.followPath(FarAutonPaths.startToLaunch);
                advanceAutonState();
                break;
            case 1:
                // Spin up and drive to launch position
                if (stateTimer.seconds() > 1.5) {
                    intake.setWantedState(Intake.IntakeWantedState.LAUNCH);
                    indexer.setWantedState(Indexer.IndexerWantedState.LAUNCH);
                    advanceAutonState(5);
                }
                break;
            case 2:
                // Launch
                if (stateTimer.seconds() > 2) {
                    launcher.setIdle();
                    intake.setWantedState(Intake.IntakeWantedState.INTAKE);
                    indexer.setWantedState(Indexer.IndexerWantedState.INTAKE);
                    follower.followPath(FarAutonPaths.launchToRow3);
                    ++numCycles;
                    advanceAutonState();
                }
                break;
            case 3:
                // Intake row 3
                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    launcher.setActive();
                    intake.setWantedState(Intake.IntakeWantedState.IDLE);
                    indexer.setWantedState(Indexer.IndexerWantedState.IDLE);
                    follower.followPath(FarAutonPaths.row3ToLaunch);
                    advanceAutonState();
                }
                break;
            case 4:
                // Drive to launch
                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    intake.setWantedState(Intake.IntakeWantedState.LAUNCH);
                    indexer.setWantedState(Indexer.IndexerWantedState.LAUNCH);
                    advanceAutonState();
                }
                break;
            case 5:
                // Launch
                if (stateTimer.seconds() > 1) {
                    launcher.setIdle();
                    intake.setWantedState(Intake.IntakeWantedState.INTAKE);
                    indexer.setWantedState(Indexer.IndexerWantedState.INTAKE);
                    follower.followPath(FarAutonPaths.launchToCorner2);
                    ++numCycles;
                    advanceAutonState();
                }
                break;
            case 6:
                // Intake corner 2
                if (!follower.isBusy() || stateTimer.seconds() > 3) {
                    launcher.setActive();
                    intake.setWantedState(Intake.IntakeWantedState.IDLE);
                    indexer.setWantedState(Indexer.IndexerWantedState.IDLE);
                    follower.followPath(FarAutonPaths.corner2ToLaunch);
                    advanceAutonState();
                }
                break;
            case 7:
                // Drive to launch
                if (!follower.isBusy() || stateTimer.seconds() > 2) {
                    intake.setWantedState(Intake.IntakeWantedState.LAUNCH);
                    indexer.setWantedState(Indexer.IndexerWantedState.LAUNCH);
                    advanceAutonState();
                }
                break;
            case 8:
                // Launch + Repeat
                if (stateTimer.seconds() > 1) {
                    launcher.setIdle();
                    intake.setWantedState(Intake.IntakeWantedState.INTAKE);
                    indexer.setWantedState(Indexer.IndexerWantedState.INTAKE);
                    follower.followPath(FarAutonPaths.launchToCorner2);
                    ++numCycles;
                    if (numCycles < 4) {
                        follower.followPath(FarAutonPaths.launchToCorner2);
                        advanceAutonState(6);
                    } else {
                        follower.followPath(FarAutonPaths.launchToCorner2);
                        advanceAutonState(9);
                    }
                }
            case 9:
                // Leave launch line and wait
                launcher.setIdle();
                intake.setWantedState(Intake.IntakeWantedState.IDLE);
                indexer.setWantedState(Indexer.IndexerWantedState.IDLE);
                break;

        }

        drivetrain.run();
        intake.run();
        launcher.run();
        indexer.run();
//        limelight.run();
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


//    public void drawOnlyCurrent() {
//        try {
//            Tuning.Drawing.drawRobot(follower.getPose());
//            Tuning.Drawing.sendPacket();
//        } catch (Exception e) {
//            throw new RuntimeException("Drawing failed " + e);
//        }
//    }
}

