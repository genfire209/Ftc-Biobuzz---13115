package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Counts the balls the Nemesis robot is holding, so the driver knows when
// it has 4 (G407: never more than 4). Two REV Color Sensor V3s on the left
// ball-path side plate look in through windows; only their distance
// reading is used (a ball in front = close):
//   - "ball_sensor_ramp" (Control Hub I2C bus 2), sensor A: where balls
//     enter the ramp. A ball passing it going IN = +1; going OUT (spit) = -1.
//   - "ball_sensor_gate" (Control Hub I2C bus 3), sensor B: where the next
//     ball waits at the gate. A ball leaving it with the gate OPEN = fired, -1.
// The sensors share one I2C address, so each needs its own bus.
//
// It's a best guess (two balls touching can pass as one), so the driver can
// reset it, and the number only drives a rumble and telemetry -- nothing is
// blocked on it. Optional: without the sensors hasSensors() is false.
// Call update() every loop.
public class BallCounter {

    public static final String RAMP_SENSOR_NAME = "ball_sensor_ramp";
    public static final String GATE_SENSOR_NAME = "ball_sensor_gate";

    public static final int FULL = 4;

    // A ball counts as "there" when the distance reading is under this.
    // TODO: TUNE on the Nemesis Setup Test COUNTER page: read the number
    // with a ball in front of each sensor and with none, pick in between.
    public static final String NEAR_FILE = "nemesis_ball_near_mm";
    public static final double DEFAULT_NEAR_MM = 40.0;

    // Readings in a row before a change counts (ignores one bad read).
    private static final int DEBOUNCE_READS = 2;
    // Intake power under this (backward) means balls are moving OUT.
    private static final double SPIT_POWER_BELOW = -0.05;
    private static final int MAX_COUNT = 8;

    private DistanceSensor ramp;
    private DistanceSensor gate;
    private double nearMm = DEFAULT_NEAR_MM;

    private int count = 0;
    private final Debounced rampSees = new Debounced();
    private final Debounced gateSees = new Debounced();
    private double rampMm = Double.NaN;
    private double gateMm = Double.NaN;

    public void init(HardwareMap hw) {
        ramp = hw.tryGet(DistanceSensor.class, RAMP_SENSOR_NAME);
        gate = hw.tryGet(DistanceSensor.class, GATE_SENSOR_NAME);
        nearMm = SavedNumber.load(NEAR_FILE, DEFAULT_NEAR_MM);
    }

    // intakePower: what the intake/ball path is running at (+ = in).
    // gateOpen: the shooter gate is open (a ball leaving B was fired).
    public void update(double intakePower, boolean gateOpen) {
        if (ramp != null) {
            rampMm = ramp.getDistance(DistanceUnit.MM);
            if (rampSees.update(rampMm < nearMm) && rampSees.on) {
                count += intakePower < SPIT_POWER_BELOW ? -1 : 1;
            }
        }
        if (gate != null) {
            gateMm = gate.getDistance(DistanceUnit.MM);
            if (gateSees.update(gateMm < nearMm) && !gateSees.on && gateOpen) {
                count -= 1;
            }
        }
        count = Math.max(0, Math.min(MAX_COUNT, count));
    }

    public int getCount() { return count; }

    public void setCount(int balls) { count = Math.max(0, Math.min(MAX_COUNT, balls)); }

    public boolean isFull() { return count >= FULL; }

    // More than allowed: spit one out now.
    public boolean isOverLimit() { return count > FULL; }

    public boolean isBallAtGate() { return gateSees.on; }

    public boolean hasSensors() { return ramp != null || gate != null; }

    public boolean hasRampSensor() { return ramp != null; }

    public boolean hasGateSensor() { return gate != null; }

    public double getRampMm() { return rampMm; }

    public double getGateMm() { return gateMm; }

    public double getNearMm() { return nearMm; }

    public void setNearMm(double mm) { nearMm = mm; }

    // On/off that only flips after DEBOUNCE_READS readings agree.
    private static class Debounced {
        boolean on = false;
        int disagree = 0;

        // Returns true when the state just flipped.
        boolean update(boolean reading) {
            if (reading == on) {
                disagree = 0;
                return false;
            }
            if (++disagree < DEBOUNCE_READS) return false;
            on = reading;
            disagree = 0;
            return true;
        }
    }
}
