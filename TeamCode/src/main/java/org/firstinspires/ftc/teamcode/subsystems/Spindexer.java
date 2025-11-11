package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.util.BallState;
import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.Pattern;
import org.firstinspires.ftc.teamcode.util.SpindexerSlot;

import lombok.Setter;

@Configurable
public class Spindexer extends Subsystem {

    // Usual subsystem variables
    private final TelemetryManager telemetry;
    private final RobotState robotState;
    // For manual override
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final HardwareMap hwMap;

    // Hardware variables
    private ServoEx servo;
    private final ColorSensors colorSensors;
    private AbsoluteAnalogEncoder encoder;

//    private DigitalChannel beamBreak;
    // Current status of the subsystem
    private double setpoint = 0;
    private SpindexerSlot currentSlot = SpindexerSlot.ONE;
    private BallState slot1State = BallState.EMPTY;
    private BallState slot2State = BallState.EMPTY;
    private BallState slot3State = BallState.EMPTY;
    private BallState lastBallState = BallState.EMPTY;
    private SpindexerState state = SpindexerState.LAUNCH;
    private boolean ballKicked = false;
    private boolean lastBallKicked = false;

    @Setter
    private FeedType feedType = FeedType.PEWPEWPEW;
    private int patternIndex = 1;   // This variable tracks which ball (1-3) of the pattern we want to shoot next
    private double currentPosition;

    public Spindexer(HardwareMap hwMap) {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.hwMap = hwMap;
        this.robotState = RobotState.getInstance();
        this.colorSensors = new ColorSensors(hwMap);
    }

    @Override
    public void init() {
        encoder = new AbsoluteAnalogEncoder(hwMap, "Analog", 3.3, AngleUnit.DEGREES);
        encoder.setReversed(false);
        servo = new ServoEx(
                hwMap,
                "Spindexer");
//        beamBreak = hwMap.get(DigitalChannel.class, "LauncherSensor");
        colorSensors.init();

        servo.getServo().getController().pwmEnable();
    }

    @Override
    public void run() {
        currentPosition = encoder.getCurrentPosition();
        // Run color sensors before logic that checks them
        colorSensors.run();
        // Based on the state, determine which spindexer slot is desired and go to it
        ballKicked = robotState.isBallKicked();

        switch (state) {
            case INTAKE:
                runIntake();
                robotState.setSpindexerAlignedForLaunch(false);
                break;
            case LAUNCH:
                runLaunch();
                robotState.setSpindexerAlignedForIntake(false);
                break;
        }

        lastBallKicked = ballKicked;
        servo.set(setpoint);
        // Update Robot State variables
        robotState.setCurrentSlot(currentSlot);
        robotState.setFull(isFull());

        updateTelemetry();
    }

    // INTAKE STATE CODE
    private void runIntake() {
        currentSlot = getNextIntakeSlot();
        setpoint = currentSlot.intakePosition;

        robotState.setSpindexerAlignedForIntake(
                Math.abs(currentPosition - currentSlot.intakeMeasurement)
                        < Tuning.SPINDEXER_ALIGNED_TOLERANCE_DEG);

        if (robotState.isSpindexerAlignedForIntake()) {
            setSlotData(currentSlot, colorSensors.getCurrentBallState());
        }
    }

    // LAUNCH STATE CODE
    private void runLaunch() {

        currentSlot = getNextLaunchSlot();
        if (robotState.isKickerSafe()) {
            setpoint = currentSlot.launchPosition;
        }

        if (robotState.isSpindexerAlignedForLaunch() && !lastBallKicked && ballKicked) {
            setSlotData(currentSlot, BallState.EMPTY);
        }
        robotState.setSpindexerAlignedForLaunch(
                Math.abs(currentPosition - currentSlot.launchMeasurement)
                        < Tuning.SPINDEXER_ALIGNED_TOLERANCE_DEG);
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addLine("--------------SPINDEXER--------------");
        telemetry.addData("State", state);
        telemetry.addData("Feed Type", feedType);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Servo Setpoint", setpoint);
        telemetry.addData("Selected Slot", currentSlot);
        telemetry.addData("Slot 1 State", slot1State);
        telemetry.addData("Slot 2 State", slot2State);
        telemetry.addData("Slot 3 State", slot3State);
        colorSensors.updateTelemetry();
    }

    @Override
    public void stop() {

    }

    public void setLaunchMode() {
        state = SpindexerState.LAUNCH;
    }

    public void setIntakeMode() {
        state = SpindexerState.INTAKE;
    }

    public void stepClockwise() {
        if (stepTimer.seconds() > 0.5) {
            currentSlot = currentSlot.last();
            stepTimer.reset();
        }
    }

    public void stepCounterClockwise() {
        if (stepTimer.seconds() > 0.5) {
            currentSlot = currentSlot.next();
            stepTimer.reset();
        }
    }

    // This function always returns the first empty intake slot
    private SpindexerSlot getNextIntakeSlot() {
        if (slot1State == BallState.EMPTY) {
            return SpindexerSlot.ONE;
        } else if (slot2State == BallState.EMPTY) {
            return SpindexerSlot.TWO;
        } else if (slot3State == BallState.EMPTY) {
            return SpindexerSlot.THREE;
        }

        return currentSlot;
    }

    /*** ALL OF THE FOLLOWING CODE is for determining which slot to use for launching. It considers primarily the feedType. ***/

    private SpindexerSlot getNextLaunchSlot() {
        // Depending on the feedType, choose a different slot to launch from next
        switch (feedType) {
            case GREEN:
                return getNextGreenSlot();
            case PURPLE:
                return getNextPurpleSlot();
            case PATTERN:
                return getNextPatternSlot();
            case PEWPEWPEW:
                // Just return the first loaded slot
                if (getSlotData(currentSlot) != BallState.EMPTY) return currentSlot;
                if (slot1State != BallState.EMPTY) {
                    return SpindexerSlot.ONE;
                } else if (slot3State != BallState.EMPTY) {
                    return SpindexerSlot.THREE;
                } else if (slot2State != BallState.EMPTY) {
                    return SpindexerSlot.TWO;
                }
        }
        // If nothing is loaded, just return the current slot
        return currentSlot;
    }

    private SpindexerSlot getNextGreenSlot() {
        if (slot1State == BallState.GREEN) {
            return SpindexerSlot.ONE;
        } else if (slot2State == BallState.GREEN) {
            return SpindexerSlot.TWO;
        } else if (slot3State == BallState.GREEN) {
            return SpindexerSlot.THREE;
        } else {
            return currentSlot;
        }
    }

    private SpindexerSlot getNextPurpleSlot() {
        if (slot1State == BallState.PURPLE) {
            return SpindexerSlot.ONE;
        } else if (slot2State == BallState.PURPLE) {
            return SpindexerSlot.TWO;
        } else if (slot3State == BallState.PURPLE) {
            return SpindexerSlot.THREE;
        } else {
            return currentSlot;
        }
    }

    private SpindexerSlot getNextPatternSlot() {
        Pattern pattern = robotState.getPattern();
        if (patternIndex == pattern.greenIndex) {
            return getNextGreenSlot();
        } else {
            return getNextPurpleSlot();
        }
    }

    public BallState getSlotData(SpindexerSlot slot) {
        switch (slot) {
            case ONE:
                return slot1State;
            case TWO:
                return slot2State;
            default:
                return slot3State;
        }
    }

    public void setSlotData(SpindexerSlot slot, BallState ballState) {
        switch (slot) {
            case ONE:
                slot1State = ballState;
                break;
            case TWO:
                slot2State = ballState;
                break;
            case THREE:
                slot3State = ballState;
                break;
        }
    }

    public void resetBallStates() {
        slot1State = BallState.EMPTY;
        slot2State = BallState.EMPTY;
        slot3State = BallState.EMPTY;
    }

    public boolean isFull() {
        return slot1State != BallState.EMPTY
                && slot2State != BallState.EMPTY
                && slot3State != BallState.EMPTY;
    }

    public boolean isEmpty() {
        return slot1State == BallState.EMPTY
                && slot2State == BallState.EMPTY
                && slot3State == BallState.EMPTY;
    }

    public boolean hasGreen() {
        return slot1State == BallState.GREEN
                || slot2State == BallState.GREEN
                || slot3State == BallState.GREEN;
    }

    public boolean hasPurple() {
        return slot1State == BallState.PURPLE
                || slot2State == BallState.PURPLE
                || slot3State == BallState.PURPLE;
    }

    private enum SpindexerState {
        INTAKE,
        LAUNCH
    }

    public enum FeedType {
        PEWPEWPEW,
        GREEN,
        PURPLE,
        PATTERN
    }
}
