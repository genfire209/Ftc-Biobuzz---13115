package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    // Defaults only: Shooter Setup Test saves the real values on the hub
    // (Square = shut, Triangle = open, D-pad left/right = pulse), and
    // init() loads them.
    public static final double GATE_CLOSED = 0.30;
    public static final double GATE_OPEN = 0.60;
    // Long enough for one ball to get past the gate, short enough that the
    // next one doesn't follow it.
    public static final double OPEN_PULSE_S = 0.25;
    public static final String GATE_CLOSED_FILE = "meet1_gate_closed";
    public static final String GATE_OPEN_FILE = "meet1_gate_open";
    public static final String OPEN_PULSE_FILE = "meet1_gate_pulse_s";
    // Shortest time the gate stays shut between balls.
    public static final double MIN_CLOSED_S = 0.30;

    // A volley gives up after this long (e.g. flywheel never got to speed).
    private static final double VOLLEY_TIMEOUT_S = 8.0;

    // MANUAL: intake power and gate set directly with setManual() (TEMP
    // TeleOp).
    public enum Mode { STOP, INTAKE, SPIT, MANUAL, SHOOT }

    private DcMotor intake;
    private Servo gate;

    private double gateClosed = GATE_CLOSED;
    private double gateOpenPosition = GATE_OPEN;
    private double openPulseS = OPEN_PULSE_S;

    private Mode mode = Mode.STOP;
    private double manualIntakePower = 0;
    private boolean manualGateOpen = false;
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

        gateClosed = savedGateClosed();
        gateOpenPosition = savedGateOpen();
        openPulseS = savedOpenPulseS();

        // Optional so a missing config entry shows up as a telemetry
        // warning instead of a crash at INIT.
        gate = hw.tryGet(Servo.class, GATE_NAME);
        closeGate();
    }

    public static double savedGateClosed() { return SavedNumber.load(GATE_CLOSED_FILE, GATE_CLOSED); }

    public static double savedGateOpen() { return SavedNumber.load(GATE_OPEN_FILE, GATE_OPEN); }

    public static double savedOpenPulseS() { return SavedNumber.load(OPEN_PULSE_FILE, OPEN_PULSE_S); }

    // STOP, INTAKE or SPIT. Cancels a volley in progress.
    public void setMode(Mode newMode) {
        if (newMode == Mode.SHOOT || newMode == Mode.MANUAL) return;   // use startVolley() / setManual()
        mode = newMode;
    }

    // Runs the intake at intakePower (+ = in) with the gate open or shut,
    // until the next setMode/startVolley. Cancels a volley in progress.
    public void setManual(double intakePower, boolean openTheGate) {
        manualIntakePower = intakePower;
        manualGateOpen = openTheGate;
        mode = Mode.MANUAL;
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
            case MANUAL:
                intake.setPower(manualIntakePower);
                if (!manualGateOpen) {
                    closeGate();
                } else if (!gateOpen) {
                    openGate();
                }
                break;
            case SHOOT:
                intake.setPower(FEED_POWER);
                if (gateOpen) {
                    if (gateTimer.seconds() > openPulseS) closeGate();
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

    // Moves whichever position the gate is in now (SHUT or OPEN) by delta,
    // right away, and saves it on the hub. Returns false if the save failed.
    public boolean nudgeGate(double delta) {
        boolean saved;
        if (gateOpen) {
            gateOpenPosition = Range.clip(gateOpenPosition + delta, 0, 1);
            saved = SavedNumber.save(GATE_OPEN_FILE, gateOpenPosition);
        } else {
            gateClosed = Range.clip(gateClosed + delta, 0, 1);
            saved = SavedNumber.save(GATE_CLOSED_FILE, gateClosed);
        }
        if (gate != null) gate.setPosition(getGatePosition());
        return saved;
    }

    public double getGatePosition() { return gateOpen ? gateOpenPosition : gateClosed; }

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
        if (gate != null) gate.setPosition(gateOpenPosition);
        gateOpen = true;
        gateTimer.reset();
    }

    private void closeGate() {
        if (gate != null) gate.setPosition(gateClosed);
        if (gateOpen) gateTimer.reset();
        gateOpen = false;
    }
}
