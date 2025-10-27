package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public enum SpindexerSlot {
    ONE(0.188, 0.745, 259, 79),
    TWO(0.56, 0.002, 16, 196),
    THREE(0.93, 0.375, 138, 318);

    public final double intakePosition;
    public final double launchPosition;
    public final double launchMeasurement;
    public final double intakeMeasurement;
    private SpindexerSlot(double intakePosition, double launchPosition, double launchMeasurement, double intakeMeasurement) {
        this.intakePosition = intakePosition;
        this.launchPosition = launchPosition;
        this.launchMeasurement = launchMeasurement;
        this.intakeMeasurement = intakeMeasurement;
    }

    public SpindexerSlot next() {
        switch (this) {
            case ONE:
                return TWO;
            case TWO:
                return THREE;
            default:
                return ONE;
        }
    }

    public SpindexerSlot last() {
        switch (this) {
            case ONE:
                return THREE;
            case TWO:
                return ONE;
            default:
                return TWO;
        }

    }
}
