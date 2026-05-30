package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.Drivetrain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

@TeleOp(name="Teleop V2", group="Competition")
public class TeleopV2 extends OpMode {
    // These two static variables will be set in the stop() method of any auton OpMode ran before this.
    private final TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    private RobotState robotState;

    private GamepadEx driver;
    private GamepadEx operator;

    // Subsystems
    private Drive drivetrain;
    private Intake intake;
    private Indexer indexer;

    private Launcher launcher;
    private Limelight limelight;

    @Override
    public void init() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (int i = 0; i < hubs.size(); ++i) {
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        robotState = RobotState.getInstance();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drivetrain = new Drive(hardwareMap, driver);
        drivetrain.init();

        intake = new Intake(hardwareMap);
        intake.init();

        indexer = new Indexer(hardwareMap);
        indexer.init();

        launcher = new Launcher(hardwareMap);
        launcher.init();

        limelight = new Limelight(hardwareMap);
        limelight.init();
        robotState.setAuton(false);
    }

    @Override
    public void init_loop() {
        if (driver.getButton(GamepadKeys.Button.B)) RobotState.getInstance().setAlliance(Alliance.RED);
        if (driver.getButton(GamepadKeys.Button.X)) RobotState.getInstance().setAlliance(Alliance.BLUE);

        telemetryManager.addLine("OpMode Initialization Completed!!!");
        telemetryManager.addData("Alliance", RobotState.getInstance().getAlliance());
        telemetryManager.update(telemetry);
    }

    @Override
    public void start() {
        drivetrain.startTeleop();
        RobotState.getInstance().setLimelightEnabled(false);
    }

    @Override
    public void loop() {
        RobotState.getInstance().addTelemetry(telemetryManager);

        processInputs();

        drivetrain.run();
        intake.run();
        indexer.run();
        launcher.run();
        limelight.run();

        // Update telemetry to panels and Driver Station
        telemetryManager.update(telemetry);
    }

    private void processInputs() {
        operator.readButtons();
        if (operator.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            indexer.setWantedState(Indexer.IndexerWantedState.EXHAUST);
            intake.setWantedState(Intake.IntakeWantedState.EXHAUST);
        } else if (operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            intake.setWantedState(Intake.IntakeWantedState.LAUNCH);
            indexer.setWantedState(Indexer.IndexerWantedState.LAUNCH);
        } else if (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            intake.setWantedState(Intake.IntakeWantedState.INTAKE);
            indexer.setWantedState(Indexer.IndexerWantedState.IDLE);
        } else {
                intake.setWantedState(Intake.IntakeWantedState.IDLE);
                indexer.setWantedState(Indexer.IndexerWantedState.IDLE);
        }

        if (gamepad2.rightTriggerWasReleased()) launcher.setIdle();

        if (operator.getButton(GamepadKeys.Button.Y) || (robotState.isIndexerLoaded() && robotState.isIntakeFull())) launcher.setActive();

        if (operator.getButton(GamepadKeys.Button.B)) launcher.setIdle();

        if (operator.getButton(GamepadKeys.Button.X)) launcher.setPreset();

        robotState.setLimelightEnabled(driver.getButton(GamepadKeys.Button.BACK));

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) intake.tweakUp();
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) intake.tweakDown();


    }
}
