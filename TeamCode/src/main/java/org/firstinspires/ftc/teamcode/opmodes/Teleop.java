package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.ButtonReader;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.security.PrivateKey;
import java.util.List;
import java.util.function.Supplier;

@TeleOp(name = "TeleOp", group = "Competition")
public class Teleop extends OpMode {

    // These two static variables will be set in the stop() method of any auton OpMode ran before this.
    private final TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    private Drive drivetrain;
    private Limelight limelight;
    private Spindexer spindexer;
    private Intake intake;
    private Launcher launcher;
    private Kicker kicker;

    private GamepadEx driver;
    private GamepadEx operator;

    @Override
    public void init() {

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (int i = 0; i < hubs.size(); ++i) {
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drivetrain = new Drive(hardwareMap, driver);
        drivetrain.init();

        limelight = new Limelight(hardwareMap);
        limelight.init();

        spindexer = new Spindexer(hardwareMap);
        spindexer.init();

        kicker = new Kicker(hardwareMap);
        kicker.init();

        intake = new Intake(hardwareMap);
        intake.init();

        launcher = new Launcher(hardwareMap);
        launcher.init();
    }

    @Override
    public void start() {
        drivetrain.startTeleop();
    }

    @Override
    public void loop() {
        RobotState.getInstance().addTelemetry(telemetryManager);
        processInputs();
        // Loop each subsystem besides follower here
        intake.run();
        spindexer.run();
        kicker.run();
        launcher.run();
        limelight.run();

        // Control drivetrain after other subsystems are run
        drivetrain.run();

        // Update telemetry to panels and Driver Station
        telemetryManager.update(telemetry);
        RobotState.getInstance().setLimelightEnabled(true);
    }

    @Override
    public void init_loop() {
        if (driver.getButton(GamepadKeys.Button.B)) RobotState.getInstance().setAlliance(Alliance.RED);
        if (driver.getButton(GamepadKeys.Button.X)) RobotState.getInstance().setAlliance(Alliance.BLUE);

        telemetryManager.addLine("OpMode Initialization Completed!!!");
        telemetryManager.addData("Alliance", RobotState.getInstance().getAlliance());
        telemetryManager.update(telemetry);
    }

    private void processInputs() {
        if (gamepad2.left_trigger > 0.1)  {
            intake.runIntake();
            kicker.resetHistory();
            spindexer.setIntakeMode();
        } else if (RobotState.getInstance().isLauncherReady()) {
            spindexer.setLaunchMode();
            intake.stopIntake();
        } else {
            intake.stopIntake();
        }

        drivetrain.setAutoAim(gamepad1.left_bumper);
        drivetrain.setRobotCentric(gamepad1.left_trigger > 0.1);
        drivetrain.setSpeed(gamepad1.right_trigger > 0.1 ? 0.35 : 1.0);

        if(gamepad2.right_bumper) {
            spindexer.stepClockwise();
        }

        if(gamepad2.left_bumper) {
            spindexer.stepCounterClockwise();
        }

        if(gamepad2.back) {
            spindexer.resetBallStates();
        }

        if(gamepad1.back) {
            RobotState.getInstance().setHeadingInitialized(false);
        }

        if (operator.getButton(GamepadKeys.Button.Y)) {
            launcher.setAuto();
        } else if (spindexer.isEmpty() && kicker.isDoneKicking()) {
            launcher.setIdle();
        }

        if (operator.getButton(GamepadKeys.Button.B)) {
            launcher.setIdle();
        }

        if (gamepad2.dpad_up) {
            launcher.setPreset();
        }

        if (gamepad2.right_trigger > 0.1) {
            spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
            spindexer.setLaunchMode();
            kicker.feed();
        } else if (gamepad2.a && spindexer.hasGreen()) {
            spindexer.setFeedType(Spindexer.FeedType.GREEN);
            spindexer.setLaunchMode();
            kicker.feed();
        } else if (gamepad2.x && spindexer.hasPurple()) {
            spindexer.setFeedType(Spindexer.FeedType.PURPLE);
            spindexer.setLaunchMode();
            kicker.feed();
        } else if (gamepad2.dpad_down) {
            spindexer.setFeedType(Spindexer.FeedType.PATTERN);
            spindexer.setLaunchMode();
            kicker.feed();
        }else {
            kicker.stopFeed();
            spindexer.setFeedType(Spindexer.FeedType.PEWPEWPEW);
        }

    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
