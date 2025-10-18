package org.firstinspires.ftc.teamcode.opmodes;

import android.widget.Button;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.security.PrivateKey;
import java.util.function.Supplier;

@TeleOp(name = "TeleOp", group = "Competition")
public class Teleop extends OpMode {

    // These two static variables will be set in the stop() method of any auton OpMode ran before this.
    private final TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    public boolean autoAimEnabled = false;
    private Drive drivetrain;
    private Limelight limelight;
    private Spindexer spindexer;
    private Intake intake;
    private Launcher launcher;

    private GamepadEx driver;
    private GamepadEx operator;

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drivetrain = new Drive(hardwareMap, driver);

        limelight = new Limelight(hardwareMap);
        limelight.init();

        spindexer = new Spindexer(hardwareMap);
        spindexer.init();

        intake = new Intake(hardwareMap);
        intake.init();

        launcher = new Launcher(hardwareMap);
        launcher.init();

        telemetryManager.addLine("OpMode Initialization Completed!!!");
        telemetryManager.update(telemetry);
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
        launcher.run();
        limelight.run();

        // Control drivetrain after other subsystems are run
        drivetrain.run();

        // Update telemetry to panels and Driver Station
        telemetryManager.update(telemetry);
    }

    private void processInputs() {
        intake.setIntake(gamepad2.left_trigger > 0.1);
        drivetrain.setAutoAim(gamepad1.right_bumper);
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
