package org.firstinspires.ftc.teamcode.subsystems;

public interface ISubsystem {

    void init();

    void periodic();

    default void stop() {}
}
