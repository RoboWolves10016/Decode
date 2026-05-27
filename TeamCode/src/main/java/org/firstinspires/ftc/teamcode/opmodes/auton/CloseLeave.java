package org.firstinspires.ftc.teamcode.opmodes.auton;

import static org.firstinspires.ftc.teamcode.opmodes.auton.CloseMoveAutonPaths.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name="Close LEAVE")
public class CloseLeave extends OpMode {
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


    @Override
    public void init() {
        drivetrain = new Drive(hardwareMap, new GamepadEx(gamepad1));
        drivetrain.init();
        drivetrain.startAuton();
        follower = drivetrain.getFollower();

        setAlliance(alliance);
        createPaths(follower);
        follower.setStartingPose(startingPose);

        launcher = new Launcher(hardwareMap);
        launcher.init();

        indexer = new Indexer(hardwareMap);
        indexer.init();

        intake = new Intake(hardwareMap);
        intake.init();

        limelight = new Limelight(hardwareMap);
        limelight.init();

        robotState.setAlliance(alliance);
        robotState.setLimelightEnabled(false);

    }

    @Override
    public void init_loop() {
//        limelight.run();
        follower.update();
        telemetryM.addData("Starting Pose", startingPose);
        telemetryM.addData("Alliance", robotState.getAlliance());
//        drawOnlyCurrent();
//        if (gamepad1.b) RobotState.getInstance().setAlliance(Alliance.RED);
//        if (gamepad1.x) RobotState.getInstance().setAlliance(Alliance.BLUE);
//        setAlliance(alliance);
//        createPaths(follower);

//        telemetryM.addData("Sees Pattern", limelight.isHasSeenPattern());
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        stateTimer.reset();
    }

    @Override
    public void loop() {
        updateTelemetry();
        RobotState.getInstance().addTelemetry(telemetryM);

        switch (autonState) {
            case 0:
                follower.setMaxPower(0.6);
                launcher.setActive();
                follower.followPath(startToLaunch);
                advanceAutonState();
                break;
            case 1:
                // Spin up and drive to launch position off line
                if (stateTimer.seconds() > 1.5) {
                    intake.setWantedState(Intake.IntakeWantedState.LAUNCH);
                    indexer.setWantedState(Indexer.IndexerWantedState.LAUNCH);
                    advanceAutonState();
                }
                break;
            case 2:
                // Launch
                if (stateTimer.seconds() > 2) {
                    launcher.setIdle();
                    intake.setWantedState(Intake.IntakeWantedState.INTAKE);
                    advanceAutonState();
                }
                break;
            case 3:
                launcher.setIdle();
                intake.setWantedState(Intake.IntakeWantedState.IDLE);
                indexer.setWantedState(Indexer.IndexerWantedState.IDLE);
                break;

        }

        drivetrain.run();
        intake.run();
        launcher.run();
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

