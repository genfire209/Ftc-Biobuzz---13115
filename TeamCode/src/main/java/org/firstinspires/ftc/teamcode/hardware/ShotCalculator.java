package org.firstinspires.ftc.teamcode.hardware;

// Distance -> target flywheel RPM, with a separate table per ball type:
// NECTAR (3.6in, 41.3g) flies differently from POLLEN (2.8in, 24.9g).
// Pure math, no hardware, independently testable.
//
// V1: empirical lookup tables, linearly interpolated. The pairs below are
// placeholders -- TODO: MEASURE once the launcher exists: fire each ball
// type from several tape-measured distances, record the RPM that lands
// it, and fill the tables in (sorted by distance ascending).
//
// V2 (future, not implemented): a physics model could extrapolate past
// the tested range, using the community BIOBUZZ shot simulator
// (github.com/LILRINO71/biobuzz-shot-sim) as a design reference.
public class ShotCalculator {

    private static final double[] POLLEN_DISTANCES_IN = {24, 48, 72, 96};
    private static final double[] POLLEN_RPMS = {0, 0, 0, 0};

    private static final double[] NECTAR_DISTANCES_IN = {24, 48, 72, 96};
    private static final double[] NECTAR_RPMS = {0, 0, 0, 0};

    // EMPTY/UNKNOWN fall back to POLLEN: every preload is POLLEN, and the
    // auto only collects POLLEN.
    public static double getTargetRpm(double distanceIn, BallSensor.BallType type) {
        if (type == BallSensor.BallType.NECTAR) {
            return interpolate(NECTAR_DISTANCES_IN, NECTAR_RPMS, distanceIn);
        }
        return interpolate(POLLEN_DISTANCES_IN, POLLEN_RPMS, distanceIn);
    }

    // True if the distance is inside the range the table was measured over
    // (outside it the RPM is clamped to the nearest end and shots get less
    // reliable).
    public static boolean isInTestedRange(double distanceIn) {
        return distanceIn >= POLLEN_DISTANCES_IN[0]
                && distanceIn <= POLLEN_DISTANCES_IN[POLLEN_DISTANCES_IN.length - 1];
    }

    private static double interpolate(double[] distances, double[] rpms, double distanceIn) {
        int last = distances.length - 1;
        if (distanceIn <= distances[0]) return rpms[0];
        if (distanceIn >= distances[last]) return rpms[last];

        for (int i = 0; i < last; i++) {
            double d0 = distances[i];
            double d1 = distances[i + 1];
            if (distanceIn >= d0 && distanceIn <= d1) {
                double t = (distanceIn - d0) / (d1 - d0);
                return rpms[i] + t * (rpms[i + 1] - rpms[i]);
            }
        }
        return rpms[last]; // Unreachable given the bounds checks above.
    }
}
