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
// SHOOT mode fires one ball at a time: the ramp pushes the balls up against
// the gate; whenever the flywheel is ready the gate opens for OPEN_PULSE_S
// (one ball), then stays shut at least MIN_CLOSED_S so the flywheel can
// recover before the next one. The ball count is a count of gate pulses --
// there's no ball sensor, so the driver is responsible for never holding
// more than 4 (G407).
//
// Call update() every loop.
public class BallPath {

    private static final String INTAKE_NAME = "intake";
    private static final String GATE_NAME = "shooter_gate";

    // TODO: VERIFY on the robot: positive power must pull balls IN and UP
    // the ramp. Flip to REVERSE if it spits them out.
    private static final DcMotor.Direction INTAKE_DIRECTION = DcMotor.Direction.FORWARD;

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

    public enum Mode { STOP, INTAKE, SPIT, SHOOT }

    private DcMotor intake;
    private Servo gate;

    private Mode mode = Mode.STOP;
    private boolean gateOpen = false;
    private int shotsFired = 0;
    private final ElapsedTime gateTimer = new ElapsedTime();

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

    public void setMode(Mode newMode) {
        mode = newMode;
    }

    // flywheelReady: Flywheel.isReady(). Only matters in SHOOT mode.
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

    // Gate pulses since the last resetShots() -- one ball per pulse if the
    // gate timing is right.
    public int getShotsFired() { return shotsFired; }

    public void resetShots() { shotsFired = 0; }

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
