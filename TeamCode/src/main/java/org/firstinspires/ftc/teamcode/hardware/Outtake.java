package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

// The Nemesis turret shooter: two motors, each on its own wheel.
//   - "launcher_pollen" (Expansion Hub motor 0): the front motor, belt to
//     the GearTrax flywheel the ball rolls along.
//   - "launcher_roller" (Expansion Hub motor 1): the rear motor, belt to the
//     top roller above the exit.
// setTarget() takes the flywheel speed (fraction of full speed, as in
// Flywheel); the roller runs at ROLLER_RATIO of that, saved on the hub by
// Nemesis Setup Test. Ready = both at speed.
//
// The roller is optional: without "launcher_roller" in the config this is
// just the flywheel, so the same code runs on the meet-1 robot.
// Call update() every loop.
public class Outtake {

    public static final String ROLLER_NAME = "launcher_roller";

    // TODO: VERIFY on the robot (Nemesis Setup Test, OUTTAKE page): the
    // roller must throw the ball OUT. Flip if it fights the flywheel.
    public static final DcMotor.Direction ROLLER_DIRECTION = DcMotor.Direction.FORWARD;

    // TODO: MEASURE like Flywheel.MAX_TICKS_PER_SEC (OUTTAKE page, left stick).
    public static final double ROLLER_MAX_TICKS_PER_SEC = 2500.0;

    // Roller speed as a fraction of the flywheel's (each of its own full
    // speed). More = more backspin from the top roller. TODO: TUNE.
    public static final String ROLLER_RATIO_FILE = "nemesis_roller_ratio";
    public static final double DEFAULT_ROLLER_RATIO = 1.0;
    public static final double MIN_ROLLER_RATIO = 0.3;
    public static final double MAX_ROLLER_RATIO = 1.5;

    private final Flywheel flywheel = new Flywheel();
    private final Flywheel roller = new Flywheel(ROLLER_NAME, ROLLER_DIRECTION, ROLLER_MAX_TICKS_PER_SEC);
    private boolean hasRoller;
    private double rollerRatio = DEFAULT_ROLLER_RATIO;

    public void init(HardwareMap hw) {
        flywheel.init(hw);
        hasRoller = roller.initIfPresent(hw);
        rollerRatio = savedRollerRatio();
    }

    public static double savedRollerRatio() {
        return Range.clip(SavedNumber.load(ROLLER_RATIO_FILE, DEFAULT_ROLLER_RATIO), MIN_ROLLER_RATIO, MAX_ROLLER_RATIO);
    }

    // Live change (Nemesis Setup Test); doesn't save it.
    public void setRollerRatio(double ratio) {
        rollerRatio = Range.clip(ratio, MIN_ROLLER_RATIO, MAX_ROLLER_RATIO);
    }

    // fraction: flywheel speed, 0..1 of its full speed.
    public void setTarget(double fraction) {
        flywheel.setTarget(fraction);
        if (hasRoller) roller.setTarget(fraction * rollerRatio);
    }

    public void stop() {
        flywheel.stop();
        if (hasRoller) roller.stop();
    }

    public void update() {
        flywheel.update();
        if (hasRoller) roller.update();
    }

    public boolean isReady() {
        return flywheel.isReady() && (!hasRoller || roller.isReady());
    }

    public boolean isSpinning() { return flywheel.isSpinning(); }

    public boolean isMaxedOut() { return flywheel.isMaxedOut() || (hasRoller && roller.isMaxedOut()); }

    // Either encoder not reading (that wheel fell back to open loop).
    public boolean isEncoderFailed() { return flywheel.isEncoderFailed() || (hasRoller && roller.isEncoderFailed()); }

    public boolean hasRoller() { return hasRoller; }

    public double getTarget() { return flywheel.getTarget(); }

    public double getMeasuredFraction() { return flywheel.getMeasuredFraction(); }

    public double getVoltage() { return flywheel.getVoltage(); }

    public Flywheel getFlywheel() { return flywheel; }

    public Flywheel getRoller() { return roller; }
}
