package org.firstinspires.ftc.teamcode.util;

public enum SpindexerSlot {
    ONE(0.205, 0.767, 251.3, 67.9),
    TWO(0.580, 0.02, 8.2, 189.85),
    THREE(0.953, 0.398, 130.0, 311.9);

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
