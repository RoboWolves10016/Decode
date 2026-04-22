package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

@TeleOp(name = "TeleOp", group = "Competition")
public class Teleop extends OpMode {

    // These two static variables will be set in the stop() method of any auton OpMode ran before this.
    private final TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    private Drive drivetrain;
//    private Limelight limelight;
    private Intake intake;

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

        intake = new Intake(hardwareMap);
        intake.init();

        RobotState.getInstance().setAuton(false);
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
        // Loop each subsystem besides follower here
        intake.run();
//        limelight.run();

        drivetrain.run();

        // Update telemetry to panels and Driver Station
        telemetryManager.update(telemetry);

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
        } else if (gamepad2.left_bumper) {
            intake.runExhaust();
        } else if (RobotState.getInstance().isLauncherReady()) {
            intake.stopIntake();
        } else {
            intake.stopIntake();
        }

        RobotState.getInstance().setLimelightEnabled(gamepad1.back);

    }

    @Override
    public void stop() {
//        limelight.stop();
    }
}
