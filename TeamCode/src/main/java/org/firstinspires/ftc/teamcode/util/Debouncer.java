package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import lombok.Getter;
import lombok.Setter;

public class Debouncer {
    public enum DebounceType {
        kRising,
        kFalling,
        kBoth
    }

    private double debounceTime;
    private DebounceType debounceType;
    private ElapsedTime timer = new ElapsedTime();
    private boolean baseline;


    public Debouncer(double debounceTime, DebounceType type) {
        this.debounceTime = debounceTime;
        this.debounceType = type;
        timer.reset();

        baseline = debounceType == DebounceType.kFalling;
    }

    private boolean hasElapsed() {
        return timer.seconds() >= debounceTime;
    }

    /**
     * Applies the debouncer to the input stream.
     *
     * @param input The current value of the input stream.
     * @return The debounced value of the input stream.
     * */
    public boolean calculate(boolean input) {
        if (input == baseline) {
            timer.reset();
        }

        if (hasElapsed()) {
            if (debounceType == DebounceType.kBoth) {
                baseline = input;
                timer.reset();
            }
            return input;
        } else {
            return baseline;
        }
    }

}
