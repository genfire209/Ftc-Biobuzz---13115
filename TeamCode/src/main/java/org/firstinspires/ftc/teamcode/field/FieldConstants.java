package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

// Fixed BIOBUZZ field geometry and AUTO poses.
//
// COORDINATES (inches, used by every Pose in this project):
//   origin = the corner where the AUDIENCE (front) wall meets the RED
//   ALLIANCE (left) wall, +x toward the blue wall, +y toward the back wall,
//   heading 0 = facing +x, counterclockwise positive (Pedro's convention).
//   The field is 144 x 144. Red = columns A-C (x < 72), blue = D-F.
//
// Everything is written for RED and converted to BLUE with forAlliance():
// the BIOBUZZ field is rotationally symmetric (manual TU03 Figure 10-2 --
// red GARDEN front-left / blue GARDEN back-right, red LOADING ZONE
// back-left / blue front-right, red starts with its audience-side CELL up
// / blue with its far CELL up), so blue = red rotated 180 deg about the
// field center.
//
// Field center is (72, 72). The perimeter walls' INSIDE faces are at
// WALL_IN = 1.3in and 144 - 1.3 = 142.7in (the tiles only cover 141.2in),
// so "touching a wall" = WALL_IN + ROBOT_HALF_IN. Tile seams are at about
// x/y = 25.0, 48.5, 72, 95.5, 119.0; columns A-F and rows 1-6 (row 1 =
// audience) are the tiles between them.
//
// Sources: "CAD" = the official BIOBUZZ field STEP export (am-5850,
// exported 2026-10-02), "Setup Guide" = 2026-2027 Event Field Setup Guide
// V1.0. Positions still marked TODO: MEASURE depend on this robot's
// intake/launcher and have to be found on a real field.
public class FieldConstants {

    public enum Alliance { RED, BLUE }

    public static final double FIELD_SIZE_IN = 144.0;
    // CAD: inside face of the perimeter walls, from the 144in grid edge.
    public static final double WALL_IN = 1.3;

    // TODO: MEASURE -- half of the robot's footprint (assumes an 18in
    // square robot). Every wall-touching pose below is offset by this.
    public static final double ROBOT_HALF_IN = 9.0;

    // ---------------- HIVE ----------------
    // CAD: red HIVE centerline (its CELLS span x 48.6-69.9). The HIVE pivots
    // about a line along x at y = 72, 43.95in up.
    public static final double RED_HIVE_X = 59.25;
    // CAD: horizontal distance from the pivot (y = 72) to the center of an
    // UPWARD CELL's tag cluster. That cluster is 49.7in up and, like the
    // whole CELL, tilted 30 deg so it faces down AND toward the end of the
    // field that CELL is on -- so each CELL is aimed at (and its tags are
    // best seen) from its own end: the audience CELL from the audience side,
    // the far CELL from the far side.
    public static final double UP_TAG_OFFSET_FROM_PIVOT_IN = 12.9;

    // Red's two CELLS, as aim points (their tag clusters when facing up).
    // RED_FIRST_CELL = audience-side CELL, up at the start (tags 34-37).
    // RED_SECOND_CELL = far-side CELL (tags 30-33), up after TIP 1.
    public static final Pose RED_FIRST_CELL = new Pose(RED_HIVE_X, 72.0 - UP_TAG_OFFSET_FROM_PIVOT_IN, 0);
    public static final Pose RED_SECOND_CELL = new Pose(RED_HIVE_X, 72.0 + UP_TAG_OFFSET_FROM_PIVOT_IN, 0);

    // AprilTag clusters on the CELL bottoms (manual Figure 9-17).
    public static final int[] RED_FIRST_CELL_TAGS = {34, 35, 36, 37};   // red audience CELL
    public static final int[] RED_SECOND_CELL_TAGS = {30, 31, 32, 33};  // red far CELL
    public static final int[] BLUE_FIRST_CELL_TAGS = {42, 43, 44, 45};  // blue far CELL
    public static final int[] BLUE_SECOND_CELL_TAGS = {38, 39, 40, 41}; // blue audience CELL

    // CAD: the A-frame's red-side floor bar runs along x 47.3-49.3 from
    // y 52.5 to 91.5 (2.1in tall), with legs rising from its ends toward the
    // pivot. Paths keep the robot's edge at least 3in left of it (x < 44).
    public static final double RED_FRAME_BAR_X_MIN = 47.3;
    public static final double RED_FRAME_BAR_Y_MIN = 52.5;
    public static final double RED_FRAME_BAR_Y_MAX = 91.5;

    // ---------------- LOADING ZONE ----------------
    // Setup Guide 8.3 + CAD: red is on tile A5, between tile seams 4 and 5
    // (y 95.5-119.0), out to x 12.9 (11in of tape from the tile edge). Blue
    // is on tile F2 (the same, rotated).
    public static final double RED_LZ_Y_MIN = 95.5;
    public static final double RED_LZ_Y_MAX = 119.0;
    // How far (in) each robot reaches into its half of the zone.
    public static final double PARK_OVERLAP_IN = 5.0;

    // ---------------- Shooting robot (red) ----------------
    // Touching the audience wall in column C, straight in front of the
    // upward CELL, facing the HIVE.
    public static final Pose RED_START = new Pose(RED_HIVE_X, WALL_IN + ROBOT_HALF_IN, Math.toRadians(90));

    // GARDEN (tile A1, Setup Guide 8.4/11.3; CAD tape at y 1.9-3.9): 4
    // POLLEN in a line along the audience wall from the corner, centers at
    // about x = 2.7, 5.5, 8.3, 11.1, y = 2.7.
    // The robot drives along the wall facing the corner with the intake
    // in front, then sweeps slowly over the balls.
    public static final Pose RED_GARDEN_APPROACH = new Pose(34.0, WALL_IN + ROBOT_HALF_IN + 1.0, Math.toRadians(180));
    // TODO: MEASURE -- where the sweep stops so the intake takes exactly 3
    // (still holding the 4th preload; G407 caps the robot at 4) or all 4.
    // As set, the robot's front edge stops just past the 3rd ball / at the
    // wall -- adjust for where your intake actually grabs.
    public static final Pose RED_GARDEN_SWEEP_END_TAKE_3 = new Pose(13.1, WALL_IN + ROBOT_HALF_IN + 1.0, Math.toRadians(180));
    public static final Pose RED_GARDEN_SWEEP_END_TAKE_4 = new Pose(10.8, WALL_IN + ROBOT_HALF_IN + 1.0, Math.toRadians(180));

    // Firing spot on the far side of the HIVE, ~38in from the far CELL's
    // tags (which face this way, ~13 deg off straight-on). The chassis
    // faces the audience wall (-90 deg) so it never has to spin near the
    // partner; the turret makes up the last ~11 deg. Kept 5in clear of red's
    // far-wall FLOWER (CAD: x 45.3-51.9, y > 136.4).
    // TODO: MEASURE -- move it to wherever your shots are most reliable.
    public static final Pose RED_BACK_FIRE = new Pose(52.0, 122.0, Math.toRadians(-90));

    // Left-wall FLOWER (Setup Guide 10.1, CAD): sticks out to x = 7.6,
    // centered at y = 48.6. Its 4 POLLEN are stacked from the tiles up inside
    // the bottom ring and come out of the bottom of the middle ring (G418B).
    // As set, the robot's front edge stops 0.5in short of it.
    // TODO: MEASURE -- how close your intake actually has to get.
    public static final Pose RED_FLOWER_PICKUP = new Pose(7.6 + ROBOT_HALF_IN + 0.5, 48.6, Math.toRadians(180));

    // ROUTE WAYPOINTS. Rule: the robot only turns in open floor -- a
    // turning 18in square sweeps a 25in circle. Between the A-frame bar
    // (x >= 47.3) and the parked partner (x <= 21.3, y 92.5-110.5) it drives
    // a straight north-south CORRIDOR at x = 33.3, square to the field and
    // facing the audience wall. Every route was checked against the CAD
    // obstacles with an 18in footprint (2026-10-02): turns clear everything
    // by >= 4.5in, the corridor clears the bar by 4.5in and the partner by
    // 3in. Re-run that check if ROBOT_HALF_IN or any pose here changes.
    public static final double RED_CORRIDOR_X = 33.3;
    // Off the start wall before turning toward the GARDEN.
    public static final Pose RED_START_EXIT = new Pose(50.0, 24.0, Math.toRadians(90));
    public static final Pose RED_CORRIDOR_SOUTH = new Pose(RED_CORRIDOR_X, 30.0, Math.toRadians(-90));
    public static final Pose RED_CORRIDOR_NORTH = new Pose(RED_CORRIDOR_X, 104.0, Math.toRadians(-90));
    // Where the robot leaves the corridor for the FLOWER, and the spot just
    // off the FLOWER where it turns to face it (and backs out to).
    public static final Pose RED_FLOWER_STAGING = new Pose(RED_CORRIDOR_X, 60.0, Math.toRadians(-90));
    public static final Pose RED_FLOWER_BACKOFF = new Pose(24.0, 48.6, Math.toRadians(180));

    // Far-wall half of the LOADING ZONE, entered from the HIVE side (2in off
    // the wall; 3.5in from the partner).
    public static final Pose RED_SHOOTER_PARK = new Pose(
            WALL_IN + ROBOT_HALF_IN + 2.0, RED_LZ_Y_MAX - PARK_OVERLAP_IN + ROBOT_HALF_IN, Math.toRadians(-90));
    // Emergency-park entry: top of the corridor at park height.
    public static final Pose RED_PARK_APPROACH = new Pose(RED_CORRIDOR_X, RED_SHOOTER_PARK.getY(), Math.toRadians(-90));

    // ---------------- Parking-only robot (red) ----------------
    // Touching the left wall just on the audience side of the LOADING ZONE
    // (starting inside it is illegal, G304E), facing the HIVE.
    public static final Pose RED_PARKER_START = new Pose(WALL_IN + ROBOT_HALF_IN, RED_LZ_Y_MIN - ROBOT_HALF_IN - 2.0, 0);
    // Audience half of the LOADING ZONE, 2in off the wall so LEAVE still
    // counts (LEAVE = not touching the perimeter wall).
    public static final Pose RED_PARKER_PARK = new Pose(WALL_IN + ROBOT_HALF_IN + 2.0, RED_LZ_Y_MIN + 6.0, 0);

    // ---------------- Helpers ----------------

    // Converts a red pose to the given alliance (blue = rotated 180 deg
    // about the field center).
    public static Pose forAlliance(Pose redPose, Alliance alliance) {
        if (alliance == Alliance.RED) return redPose;
        return new Pose(
                FIELD_SIZE_IN - redPose.getX(),
                FIELD_SIZE_IN - redPose.getY(),
                normalizeRad(redPose.getHeading() + Math.PI));
    }

    public static int[] firstCellTags(Alliance alliance) {
        return (alliance == Alliance.RED) ? RED_FIRST_CELL_TAGS : BLUE_FIRST_CELL_TAGS;
    }

    public static int[] secondCellTags(Alliance alliance) {
        return (alliance == Alliance.RED) ? RED_SECOND_CELL_TAGS : BLUE_SECOND_CELL_TAGS;
    }

    public static double distanceToHive(Pose robotPose, Pose cell) {
        return Math.hypot(cell.getX() - robotPose.getX(), cell.getY() - robotPose.getY());
    }

    // ---------------- CELL geometry (manual 9.6.2, Figures 9-9 to 9-11), inches ----------------
    public static final double CELL_OPENING_WIDTH_IN = 20.0;
    public static final double CELL_OPENING_HEIGHT_IN = 14.0;
    public static final double CELL_DEPTH_IN = 12.0;
    public static final double CELL_BOTTOM_OPENING_ABOVE_TILES_IN = 53.5;
    public static final double CELL_TOP_OPENING_ABOVE_TILES_IN = 65.6;
    public static final double HIVE_PIVOT_HEIGHT_ABOVE_TILES_IN = 43.95;
    public static final double HIVE_ARM_TILT_DEG = 30.0;

    private static double normalizeRad(double rad) {
        while (rad < 0) rad += 2 * Math.PI;
        while (rad >= 2 * Math.PI) rad -= 2 * Math.PI;
        return rad;
    }
}
