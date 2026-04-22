package org.firstinspires.ftc.teamcode.util;

public enum SpindexerSlot {
    ONE(0.245, 0.790, 257.9, 81.1),
    TWO(0.610, 0.050, 18.6, 199.8),
    THREE(0.985, 0.428, 140.5, 321.8);

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
