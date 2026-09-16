package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

// Flywheel launcher hardware: non-blocking ramped-RPM velocity control.
// update() is the ONLY place that calls setVelocity() and must be called
// every loop() iteration -- never gate it behind Thread.sleep, or the
// Pedro Follower driving the robot will stall along with it.
//
// Hardware not built yet: motor names, count (1 hooded wheel vs. 2
// flywheels), TICKS_PER_REV, and RAMP_RPM_PER_SEC below are all
// placeholders -- TODO: fill in once the launcher exists (see
// systemTest/LauncherBenchTest.java for the bench-tuning procedure).
public class Launcher {

    // TODO: MEASURE/DECIDE -- device names, must match Driver Hub config.
    private static final String LEFT_MOTOR_NAME = "launcher_left";
    private static final String RIGHT_MOTOR_NAME = "launcher_right";

    // TODO: MEASURE -- encoder ticks per output-shaft revolution for the
    // chosen launcher motor.
    private static final double TICKS_PER_REV = 28.0;

    // TODO: MEASURE -- how fast the commanded RPM is allowed to ramp,
    // to avoid slamming the flywheel target and inducing oscillation.
    private static final double RAMP_RPM_PER_SEC = 4000.0;

    // Fraction of target RPM within which the launcher is considered
    // "ready to fire". Safe default; retune once real hardware exists.
    private static final double READY_BAND = 0.05;

    private DcMotorEx left;
    private DcMotorEx right;

    private double targetRpm = 0.0;
    private double commandedRpm = 0.0;
    private long lastNs = 0L;

    public void init(HardwareMap hw) {
        left = hw.get(DcMotorEx.class, LEFT_MOTOR_NAME);
        right = hw.get(DcMotorEx.class, RIGHT_MOTOR_NAME);

        right.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // TODO: TUNE -- PIDF coefficients for the real launcher motor,
            // once bench-tested via LauncherBenchTest.
            PIDFCoefficients pidf = m.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            pidf.p = 10.0;
            pidf.i = 0.0;
            pidf.d = 0.0;
            pidf.f = 11.7;
            m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }

        targetRpm = 0.0;
        commandedRpm = 0.0;
        lastNs = 0L;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    // Call every loop() iteration -- ramps commandedRpm toward targetRpm
    // and applies it as a velocity setpoint. Non-blocking.
    public void update() {
        long now = System.nanoTime();
        double dt = (lastNs == 0L) ? 0.02 : (now - lastNs) / 1e9;
        lastNs = now;
        dt = Range.clip(dt, 0.005, 0.05);

        double step = RAMP_RPM_PER_SEC * dt;
        commandedRpm += Range.clip(targetRpm - commandedRpm, -step, step);

        double ticksPerSec = rpmToTicksPerSec(commandedRpm);
        left.setVelocity(ticksPerSec);
        right.setVelocity(ticksPerSec);
    }

    public boolean isReadyToFire() {
        if (targetRpm <= 0.0) return false;
        double measuredRpm = ticksPerSecToRpm(left.getVelocity());
        return Math.abs(measuredRpm - targetRpm) <= READY_BAND * targetRpm;
    }

    public double getMeasuredRpm() {
        return ticksPerSecToRpm(left.getVelocity());
    }

    public void stop() {
        targetRpm = 0.0;
        commandedRpm = 0.0;
        left.setVelocity(0);
        right.setVelocity(0);
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private static double ticksPerSecToRpm(double ticksPerSec) {
        return (ticksPerSec * 60.0) / TICKS_PER_REV;
    }
}
