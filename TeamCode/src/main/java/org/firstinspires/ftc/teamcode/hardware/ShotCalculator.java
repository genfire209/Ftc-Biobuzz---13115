package org.firstinspires.ftc.teamcode.hardware;

// Distance -> target flywheel RPM. Pure math, no HardwareMap/motor
// references, independently testable.
//
// V1: empirical lookup table, linearly interpolated. No launcher exists
// yet, so the (distance, rpm) pairs below are placeholders -- TODO:
// replace with real measured data once the launcher is built (fire at
// several measured distances, record the RPM that lands the shot, fill
// this table in).
//
// V2 (future, not implemented): a physics-based model could extrapolate
// beyond the table's tested range, referencing the community BIOBUZZ shot
// simulator (github.com/LILRINO71/biobuzz-shot-sim): exit ball speed =
// eta * flywheel surface speed, with eta = 0.45 for a single hooded wheel
// or 0.90 (average) for two flywheels; quadratic air drag (C_D = 0.45);
// backspin lift (C_L = 0.20 * min(spin ratio, 1)); motor top speed =
// free speed * 0.97 * (battery voltage / 12V). Treat as a design
// reference, not validated constants for this robot's launcher.
public class ShotCalculator {

    // TODO: MEASURE -- replace with real (distance_in, rpm) pairs from
    // bench/field testing once the launcher exists. Must stay sorted by
    // distance ascending.
    private static final double[] DISTANCES_IN = {24, 48, 72, 96};
    private static final double[] RPMS = {0, 0, 0, 0};

    public static double getTargetRpm(double distanceIn) {
        if (distanceIn <= DISTANCES_IN[0]) {
            return RPMS[0];
        }
        int last = DISTANCES_IN.length - 1;
        if (distanceIn >= DISTANCES_IN[last]) {
            return RPMS[last];
        }

        for (int i = 0; i < last; i++) {
            double d0 = DISTANCES_IN[i];
            double d1 = DISTANCES_IN[i + 1];
            if (distanceIn >= d0 && distanceIn <= d1) {
                double t = (distanceIn - d0) / (d1 - d0);
                return RPMS[i] + t * (RPMS[i + 1] - RPMS[i]);
            }
        }

        // Unreachable given the bounds checks above.
        return RPMS[last];
    }
}
