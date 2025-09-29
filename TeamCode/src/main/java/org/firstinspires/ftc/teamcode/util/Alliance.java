package org.firstinspires.ftc.teamcode.util;

public enum Alliance {
    RED(0),
    BLUE(180);

    public final double driverForwardHeading;

    Alliance(double forwardDirection) {
        this.driverForwardHeading = forwardDirection;
    }
}
