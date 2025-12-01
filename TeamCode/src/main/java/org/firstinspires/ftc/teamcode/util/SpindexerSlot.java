package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public enum SpindexerSlot {
    ONE(0.180, 0.742, 258.4, 75.6),
    TWO(0.552, 0.00, 17, 196.4),
    THREE(0.922, 0.371, 137.3, 316.85);

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
