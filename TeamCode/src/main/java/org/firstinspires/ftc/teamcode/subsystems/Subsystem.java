package org.firstinspires.ftc.teamcode.subsystems;

public abstract class Subsystem {

    abstract void init();

    abstract void run();
    protected abstract void updateTelemetry();

    abstract void stop();
}
