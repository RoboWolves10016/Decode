package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.function.Supplier;

@TeleOp(name = "TeleOp", group = "Competition")
public class Teleop extends OpMode {

    // These two static variables will be set in the stop() method of any auton OpMode ran before this.
    public static Pose startingPose;
    public static Alliance startingAlliance = Alliance.BLUE;

    private final TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    public boolean autoAimEnabled = false;
    public Supplier<Double> rotationSupplier;
    private Follower follower;
    private Limelight limelight;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        limelight = new Limelight(hardwareMap, telemetry);
        limelight.init();

        rotationSupplier = () -> autoAimEnabled ? 0d : -gamepad1.right_stick_x;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Loop each subsystem besides follower here
        limelight.periodic();


        // Control drivetrain after other subsystems are run
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                rotationSupplier.get(),
                false);

        // Update telemetry to panels and Driver Station
        telemetryManager.update(telemetry);
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
