package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

public abstract class AutonBase extends OpMode {
    protected final RobotState robotState = RobotState.getInstance();
    protected final ElapsedTime stateTimer = new ElapsedTime();

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    protected Drive drivetrain;
    protected Follower follower;
    protected Launcher launcher;
    protected Intake intake;
    protected Indexer indexer;
    protected Limelight limelight;
    protected int autonState = 0;

    @Override
    public void init() { // DO NOT OVERRIDE
        drivetrain = new Drive(hardwareMap, new GamepadEx(gamepad1));
        drivetrain.init();
        drivetrain.startAuton();

        launcher = new Launcher(hardwareMap);
        launcher.init();

        intake = new Intake(hardwareMap);
        intake.init();

        indexer = new Indexer(hardwareMap);
        indexer.init();

        limelight = new Limelight(hardwareMap);
        limelight.init();

        follower = drivetrain.getFollower();
    }

    @Override
    public void init_loop() { // DO NOT OVERRIDE
        follower.update();
        telemetry.addData("Alliance", robotState.getAlliance());
        if (gamepad1.b) RobotState.getInstance().setAlliance(Alliance.RED);
        if (gamepad1.x) RobotState.getInstance().setAlliance(Alliance.BLUE);
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        stateTimer.reset();
    }

    @Override
    public void loop() {
        pushTelemetry();
        drivetrain.run();
        intake.run();
        launcher.run();
        indexer.run();
        limelight.run();
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        Flywheel.useManualRpm = false;
        Hood.useManualOverride = false;
    }

    protected void spinUp() {
        launcher.setActive();
    }

    protected void spinDown() {
        launcher.setIdle();
    }

    protected void intake() {
        intake.setWantedState(Intake.IntakeWantedState.INTAKE);
//        indexer.setWantedState(Indexer.IndexerWantedState.INTAKE);
    }

    protected void launch() {
        intake.setWantedState(Intake.IntakeWantedState.LAUNCH);
        indexer.setWantedState(Indexer.IndexerWantedState.LAUNCH);
    }

    protected void forceLaunch() {
        launch();
        indexer.forceFeed();
        intake.forceFeed();
    }

    protected void stopForceLaunch() {
        stopLaunch();
        indexer.stopForceFeed();
        intake.stopForceFeed();
    }
    protected void stopLaunch() {
        intake.setWantedState(Intake.IntakeWantedState.IDLE);
        indexer.setWantedState(Indexer.IndexerWantedState.IDLE);
    }

    protected void advanceState() {
        autonState = autonState + 1;
        stateTimer.reset();
    }

    protected void advanceState(int newState) {
        autonState = newState;
        stateTimer.reset();
    }

    private void pushTelemetry() {
        telemetryM.addLine("---------AUTON---------");
        telemetryM.addData("State Duration", stateTimer.seconds());
        telemetryM.addData("Auton State", autonState);
        telemetryM.addData("Is Busy", follower.isBusy());
        telemetryM.addData("Follower Pose", follower.getPose());
        telemetryM.addData("T-Value", follower.getCurrentTValue());
        telemetryM.addData("Path Number", follower.getCurrentPathNumber());

    }

}
