package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

// Fixed BIOBUZZ field geometry (from the V1 Competition Manual, Section 9
// ARENA). HIVE_TARGET poses are placeholders — TODO: MEASURE against the
// real field once a coordinate origin is picked (same origin used for
// startPose in the auto OpModes).
public class FieldConstants {

    public enum Alliance { RED, BLUE }

    // TODO: MEASURE — tape-measure the HIVE CELL entrance against the field
    // coordinate origin. The two HIVEs are ~25.5in apart center-to-center,
    // both near the middle of the 144in x 144in field (manual Fig. 9-10).
    public static final Pose RED_HIVE_TARGET = new Pose(72, 72, 0);
    public static final Pose BLUE_HIVE_TARGET = new Pose(72, 72, 0);

    // CELL geometry (manual Section 9.6.2 / Figure 9-9, 9-10, 9-11), inches.
    public static final double CELL_OPENING_WIDTH_IN = 20.0;
    public static final double CELL_OPENING_HEIGHT_IN = 14.0;
    public static final double CELL_DEPTH_IN = 12.0;
    public static final double CELL_BOTTOM_OPENING_ABOVE_TILES_IN = 53.5;
    public static final double CELL_TOP_OPENING_ABOVE_TILES_IN = 65.6;
    public static final double HIVE_PIVOT_HEIGHT_ABOVE_TILES_IN = 43.95;
    public static final double HIVE_ARM_TILT_DEG = 30.0;

    public static double distanceToHive(Pose robotPose, Pose hiveTarget) {
        return Math.hypot(hiveTarget.getX() - robotPose.getX(), hiveTarget.getY() - robotPose.getY());
    }
}
