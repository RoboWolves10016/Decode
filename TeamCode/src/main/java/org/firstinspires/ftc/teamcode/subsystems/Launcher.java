package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotState;

@Configurable
public class Launcher extends Subsystem {
    // Required subsystem components
    private final TelemetryManager telemetry;
    private final HardwareMap hwMap;
    private final RobotState robotState = RobotState.getInstance();

    private Flywheel flywheel;
    private Hood hood;
    private Turret turret;

    private enum LauncherState {
        IDLE,
        SPINNING_UP,
        SPUN_UP,
    }

    private enum LauncherWantedState {
        IDLE,
        ACTIVE
    }

    private LauncherState currentState = LauncherState.IDLE;
    private LauncherWantedState wantedState = LauncherWantedState.IDLE;

    public Launcher(HardwareMap hwMap) {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.hwMap = hwMap;
    }
    @Override
    public void init() {
        flywheel = new Flywheel(hwMap);
        flywheel.init();

        hood = new Hood(hwMap);
        hood.init();

        turret = new Turret(hwMap);
        turret.init();
    }

    @Override
    public void run() {
        switch (wantedState) {
            case IDLE:
                flywheel.setIdle();
                hood.setTracking(false);
                turret.setTracking(false);
                break;
            case ACTIVE:
                flywheel.setAuto();
                hood.setTracking(true);
                turret.setTracking(true);
                break;
        }

        robotState.setLauncherReady(turret.isAligned() && flywheel.isReady());

        flywheel.run();
        hood.run();
        turret.run();
        updateTelemetry();
    }

    public void setActive() {
        wantedState = LauncherWantedState.ACTIVE;
    }

    public void setIdle() {
        wantedState = LauncherWantedState.IDLE;
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------Master Launcher--------------");
        telemetry.addData("State", currentState.toString());
        flywheel.updateTelemetry();
        hood.updateTelemetry();
        turret.updateTelemetry();
    }

    @Override
    public void stop() {
        flywheel.stop();
        hood.stop();
        turret.stop();
    }
}
