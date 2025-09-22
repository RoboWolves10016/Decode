package org.firstinspires.ftc.teamcode.subsystems;

public interface ISubsystem {

    public abstract void init();

    public abstract void periodic();

    public default void stop() {}
}
