package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// The meet-1 shooter flywheel: one Matrix 12V motor ("launcher_pollen",
// Expansion Hub motor port 0, encoder cable in encoder port 0).
//
// Speeds are a FRACTION OF FULL SPEED (0..1), so the same preset numbers
// work whether or not the encoder is working:
//   - Encoder working: feed-forward + P on the measured ticks/s, with
//     battery-voltage compensation. Ready = within READY_BAND of the target
//     for a few loops in a row.
//   - Encoder not reading (unplugged, broken, or wrong port): after
//     ENCODER_FAIL_S of spinning with ~0 measured speed it switches for the
//     rest of the OpMode to voltage-compensated open loop, and "ready" just
//     means it has been spinning for OPEN_LOOP_SPINUP_S. Shots get less
//     consistent, but the robot still shoots.
//
// Runs in RUN_WITHOUT_ENCODER and does its own speed loop, so the motor
// type chosen in the robot config doesn't matter. Call update() every loop.
public class Flywheel {

    public static final String MOTOR_NAME = "launcher_pollen";

    // On the robot (Intake + Outtake Test, 2026-10-10) REVERSE spun the
    // outtake backward, so FORWARD. (The 2026-10-04 bench rig was mounted
    // differently and needed REVERSE.)
    public static final DcMotor.Direction DIRECTION = DcMotor.Direction.FORWARD;

    // TODO: MEASURE with systemTest/ShooterSetupTest: highest ticks/s at full
    // power with a charged battery and no ball. Fraction 1.0 = this speed.
    public static final double MAX_TICKS_PER_SEC = 2500.0;

    // Battery voltage the feed-forward is scaled to.
    private static final double NOMINAL_VOLTAGE = 12.5;
    // Extra power per unit of speed error (error as a fraction of full speed).
    private static final double KP = 1.5;

    // Ready to shoot when within this fraction of full speed of the target...
    private static final double READY_BAND = 0.03;
    // ...for this many loops in a row (ignores one lucky reading).
    private static final int READY_LOOPS = 3;

    // Encoder-fail detection: commanded at least this fast, but measured
    // under ENCODER_ALIVE for this long -> treat the encoder as dead.
    private static final double ENCODER_CHECK_MIN_TARGET = 0.2;
    private static final double ENCODER_ALIVE = 0.05;
    private static final double ENCODER_FAIL_S = 1.0;

    // Open-loop fallback: how long it takes to spin up from rest.
    private static final double OPEN_LOOP_SPINUP_S = 1.5;

    // If the target is faster than the flywheel can go (MAX_TICKS_PER_SEC
    // set too high, or a flat battery) it sits at full power and never gets
    // in the band. After this long at full power, call it ready anyway so
    // the robot still shoots.
    private static final double SATURATED_READY_S = 1.5;

    // Reading the battery takes a hub round trip, so not every loop.
    private static final double VOLTAGE_READ_PERIOD_S = 0.25;

    private DcMotorEx motor;
    private VoltageSensor battery;

    private double target = 0;
    private double measuredTicksPerSec = 0;
    private double power = 0;
    private double voltage = NOMINAL_VOLTAGE;
    private int readyLoops = 0;
    private boolean encoderFailed = false;

    private final ElapsedTime spinTimer = new ElapsedTime();
    private final ElapsedTime deadEncoderTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime unsaturatedTimer = new ElapsedTime();

    public void init(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(DIRECTION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        battery = hw.voltageSensor.iterator().hasNext() ? hw.voltageSensor.iterator().next() : null;
        readVoltage();
    }

    // fraction: 0..1 of full speed. Restarts the spin-up clock only when the
    // flywheel was stopped, so trimming a preset doesn't reset "ready".
    public void setTarget(double fraction) {
        double clipped = Range.clip(fraction, 0, 1);
        if (target <= 0 && clipped > 0) {
            spinTimer.reset();
            deadEncoderTimer.reset();
            unsaturatedTimer.reset();
        }
        target = clipped;
    }

    public void stop() {
        target = 0;
        readyLoops = 0;
    }

    public void update() {
        if (voltageTimer.seconds() > VOLTAGE_READ_PERIOD_S) readVoltage();

        // abs(): the encoder's polarity on this motor is unverified, and the
        // flywheel only ever spins one way.
        measuredTicksPerSec = Math.abs(motor.getVelocity());
        double measured = measuredTicksPerSec / MAX_TICKS_PER_SEC;

        if (target <= 0) {
            power = 0;
            readyLoops = 0;
            unsaturatedTimer.reset();
            motor.setPower(0);
            return;
        }

        if (!encoderFailed) {
            boolean looksDead = target >= ENCODER_CHECK_MIN_TARGET && measured < ENCODER_ALIVE;
            if (!looksDead) {
                deadEncoderTimer.reset();
            } else if (deadEncoderTimer.seconds() > ENCODER_FAIL_S) {
                encoderFailed = true;
            }
        }

        double feedForward = target * NOMINAL_VOLTAGE / voltage;
        double correction = encoderFailed ? 0 : KP * (target - measured);
        power = Range.clip(feedForward + correction, 0, 1);
        motor.setPower(power);
        if (power < 1.0) unsaturatedTimer.reset();

        boolean inBand = !encoderFailed && Math.abs(measured - target) < READY_BAND;
        readyLoops = inBand ? readyLoops + 1 : 0;
    }

    public boolean isReady() {
        if (target <= 0) return false;
        if (encoderFailed) return spinTimer.seconds() > OPEN_LOOP_SPINUP_S;
        return readyLoops >= READY_LOOPS || isMaxedOut();
    }

    public boolean isSpinning() { return target > 0; }

    // At full power for a while and still short of the target.
    public boolean isMaxedOut() {
        return target > 0 && power >= 1.0 && unsaturatedTimer.seconds() > SATURATED_READY_S;
    }

    public boolean isEncoderFailed() { return encoderFailed; }

    public double getTarget() { return target; }

    public double getMeasuredFraction() { return measuredTicksPerSec / MAX_TICKS_PER_SEC; }

    public double getMeasuredTicksPerSec() { return measuredTicksPerSec; }

    public double getPower() { return power; }

    public double getVoltage() { return voltage; }

    private void readVoltage() {
        voltageTimer.reset();
        if (battery == null) return;
        double v = battery.getVoltage();
        // Ignore a bad read (0 V) rather than dividing by it.
        if (v > 6.0) voltage = v;
    }
}
