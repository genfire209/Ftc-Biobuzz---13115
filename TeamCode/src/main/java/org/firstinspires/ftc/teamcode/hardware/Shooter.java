package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Fires a volley one ball at a time: for each ball, wait until the
// flywheel is at the RPM for this distance and ball type AND the turret
// is aimed, pulse the feeder, then let the flywheel recover before the
// next ball. Non-blocking -- call update() every loop.
//
// Also pre-spins the flywheel between volleys so the first shot at the
// next firing spot doesn't wait on a cold flywheel.
public class Shooter {

    // TODO: DECIDE/MEASURE -- feeder device name and how long one pulse
    // takes to push exactly one ball into the flywheel.
    private static final String FEEDER_MOTOR_NAME = "feeder";
    private static final double FEEDER_POWER = 0.8;
    private static final double FEED_PULSE_S = 0.25;

    // With a ball sensor fitted: how long the feed slot must read EMPTY
    // before the volley ends early (gives the next ball time to roll in).
    private static final double EMPTY_CONFIRM_S = 0.3;

    private final Launcher launcher = new Launcher();
    private final BallSensor ballSensor = new BallSensor();
    private DcMotor feeder;

    private enum State { IDLE, WAIT_READY, FEEDING }
    private State state = State.IDLE;
    private final Timer timer = new Timer();
    private final Timer emptyTimer = new Timer();
    private boolean slotWasEmpty = false;

    private int ballsToFire = 0;
    private int ballsFired = 0;
    private BallSensor.BallType nextBall = BallSensor.BallType.UNKNOWN;

    public void init(HardwareMap hw) {
        launcher.init(hw);
        ballSensor.init(hw);
        feeder = hw.get(DcMotor.class, FEEDER_MOTOR_NAME);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Spin the flywheel toward the RPM for a shot from this distance
    // (assumes POLLEN). Ignored while a volley is running.
    public void preSpin(double distanceIn) {
        if (state == State.IDLE) {
            launcher.setTargetRpm(ShotCalculator.getTargetRpm(distanceIn, BallSensor.BallType.POLLEN));
        }
    }

    public void startVolley(int balls) {
        ballsToFire = balls;
        ballsFired = 0;
        slotWasEmpty = false;
        state = (balls > 0) ? State.WAIT_READY : State.IDLE;
    }

    // Stops a volley early (e.g. out of time). Leaves the flywheel spinning.
    public void abortVolley() {
        feeder.setPower(0);
        state = State.IDLE;
    }

    // aimed = the turret is on target (or the caller gave up waiting on aim).
    public void update(double distanceIn, boolean aimed) {
        launcher.update();

        switch (state) {
            case WAIT_READY:
                nextBall = ballSensor.read();
                if (nextBall == BallSensor.BallType.EMPTY) {
                    if (!slotWasEmpty) {
                        slotWasEmpty = true;
                        emptyTimer.resetTimer();
                    } else if (emptyTimer.getElapsedTimeSeconds() > EMPTY_CONFIRM_S) {
                        state = State.IDLE; // nothing left to fire
                    }
                    break;
                }
                slotWasEmpty = false;

                launcher.setTargetRpm(ShotCalculator.getTargetRpm(distanceIn, nextBall));
                if (aimed && launcher.isReadyToFire()) {
                    feeder.setPower(FEEDER_POWER);
                    timer.resetTimer();
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                if (timer.getElapsedTimeSeconds() > FEED_PULSE_S) {
                    feeder.setPower(0);
                    ballsFired++;
                    state = (ballsFired >= ballsToFire) ? State.IDLE : State.WAIT_READY;
                }
                break;

            case IDLE:
                break;
        }
    }

    public boolean isVolleyDone() { return state == State.IDLE; }

    public int getBallsFired() { return ballsFired; }

    public BallSensor.BallType getNextBall() { return nextBall; }

    public boolean hasBallSensor() { return ballSensor.isPresent(); }

    public double getMeasuredRpm() { return launcher.getMeasuredRpm(); }

    public void stop() {
        feeder.setPower(0);
        launcher.stop();
        state = State.IDLE;
    }
}
