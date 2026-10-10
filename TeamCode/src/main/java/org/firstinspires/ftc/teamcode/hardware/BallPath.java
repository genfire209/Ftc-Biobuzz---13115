package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Everything between the floor and the flywheel on the meet-1 robot:
//   - "intake" (Expansion Hub motor port 3, goBILDA 1150 RPM): ONE motor
//     runs the front vector wheels AND the ramp rollers.
//   - "shooter_gate" (Control Hub servo port 1): a stopper that holds the
//     balls back from the flywheel until it's time to shoot.
//
// startVolley(n) fires n balls back-to-back with no further input: the ramp
// pushes the balls up against the gate; each time the flywheel is at speed
// the gate opens for OPEN_PULSE_S (one ball), then stays shut at least
// MIN_CLOSED_S while the flywheel recovers. After n gate pulses (or
// VOLLEY_TIMEOUT_S) it stops by itself. Waiting for speed between balls is
// what makes balls 2-4 fly as far as ball 1 -- opening the gate once and
// letting all four through would slow the single flywheel and drop the
// later ones short.
//
// The ball count is a count of gate pulses -- there's no ball sensor, so
// the driver is responsible for never holding more than 4 (G407).
//
// Call update() every loop.
public class BallPath {

    public static final String INTAKE_NAME = "intake";
    private static final String GATE_NAME = "shooter_gate";

    // TODO: VERIFY on the robot: positive power must pull balls IN and UP
    // the ramp. Flip to REVERSE if it spits them out.
    public static final DcMotor.Direction INTAKE_DIRECTION = DcMotor.Direction.FORWARD;

    public static final double INTAKE_POWER = 1.0;
    // While shooting: pushes the next ball up against the gate.
    public static final double FEED_POWER = 0.8;
    public static final double SPIT_POWER = -0.6;

    // TODO: MEASURE with systemTest/ShooterSetupTest.
    public static final double GATE_CLOSED = 0.30;
    public static final double GATE_OPEN = 0.60;
    // Long enough for one ball to get past the gate, short enough that the
    // next one doesn't follow it.
    public static final double OPEN_PULSE_S = 0.25;
    // Shortest time the gate stays shut between balls.
    public static final double MIN_CLOSED_S = 0.30;

    // A volley gives up after this long (e.g. flywheel never got to speed).
    private static final double VOLLEY_TIMEOUT_S = 8.0;

    public enum Mode { STOP, INTAKE, SPIT, SHOOT }

    private DcMotor intake;
    private Servo gate;

    private Mode mode = Mode.STOP;
    private boolean gateOpen = false;
    private int shotsFired = 0;
    private int volleySize = 0;
    private final ElapsedTime gateTimer = new ElapsedTime();
    private final ElapsedTime volleyTimer = new ElapsedTime();

    public void init(HardwareMap hw) {
        intake = hw.get(DcMotor.class, INTAKE_NAME);
        intake.setDirection(INTAKE_DIRECTION);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Optional so a missing config entry shows up as a telemetry
        // warning instead of a crash at INIT.
        gate = hw.tryGet(Servo.class, GATE_NAME);
        closeGate();
    }

    // STOP, INTAKE or SPIT. Cancels a volley in progress.
    public void setMode(Mode newMode) {
        if (newMode == Mode.SHOOT) return;   // use startVolley()
        mode = newMode;
    }

    // Fires `balls` gate pulses, each one as soon as the flywheel is at speed.
    // Spin the flywheel up at the same time (Flywheel.setTarget).
    public void startVolley(int balls) {
        volleySize = balls;
        shotsFired = 0;
        volleyTimer.reset();
        mode = Mode.SHOOT;
    }

    public boolean isVolleyActive() { return mode == Mode.SHOOT; }

    // flywheelReady: Flywheel.isReady(). Only matters during a volley.
    public void update(boolean flywheelReady) {
        switch (mode) {
            case INTAKE:
                intake.setPower(INTAKE_POWER);
                closeGate();
                break;
            case SPIT:
                intake.setPower(SPIT_POWER);
                closeGate();
                break;
            case SHOOT:
                intake.setPower(FEED_POWER);
                if (gateOpen) {
                    if (gateTimer.seconds() > OPEN_PULSE_S) closeGate();
                } else if (shotsFired >= volleySize || volleyTimer.seconds() > VOLLEY_TIMEOUT_S) {
                    mode = Mode.STOP;            // volley finished
                    intake.setPower(0);
                } else if (flywheelReady && gateTimer.seconds() > MIN_CLOSED_S) {
                    openGate();
                    shotsFired++;
                }
                break;
            case STOP:
            default:
                intake.setPower(0);
                closeGate();
                break;
        }
    }

    public Mode getMode() { return mode; }

    public boolean isGateOpen() { return gateOpen; }

    public boolean hasGate() { return gate != null; }

    // Gate pulses in the current/last volley -- one ball per pulse if the
    // gate timing is right.
    public int getShotsFired() { return shotsFired; }

    public int getVolleySize() { return volleySize; }

    public void stop() {
        mode = Mode.STOP;
        intake.setPower(0);
        closeGate();
    }

    private void openGate() {
        if (gate != null) gate.setPosition(GATE_OPEN);
        gateOpen = true;
        gateTimer.reset();
    }

    private void closeGate() {
        if (gate != null) gate.setPosition(GATE_CLOSED);
        if (gateOpen) gateTimer.reset();
        gateOpen = false;
    }
}
