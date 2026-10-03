package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.CameraTilt;
import org.firstinspires.ftc.teamcode.hardware.Turret;

// Points the turret (launcher + Limelight) at a CELL in two stages:
//   1. Coarse: from Pedro's robot pose and the CELL's field position, so
//      the tags are already in frame without a search.
//   2. Fine: once the CELL's tags are visible, it re-centers on tx every
//      loop, so the shot lines up with the real CELL even if odometry has
//      drifted.
// Distance comes from ty when the tags are visible, otherwise from
// odometry. Call update() every loop; it never blocks.
public class HiveAimer {

    // CAD: center of an UPWARD CELL's tag cluster above the tiles. (The
    // cluster is tilted 30 deg toward that CELL's end of the field, so from
    // the firing spots it's seen 13-22 deg off straight-on.)
    private static final double TAG_HEIGHT_ABOVE_TILES_IN = 49.7;

    // TODO: MEASURE -- Limelight lens height above the tiles once it's on
    // the turret (aim for 8-12in: lower sees the tags at a better angle).
    private static final double CAMERA_HEIGHT_ABOVE_TILES_IN = 10.0;

    // TODO: VERIFY -- turret degrees to add per degree of tx. Turret angle
    // is counterclockwise-positive and tx is positive when the tag is right
    // of the crosshair, so turning right (negative) centers it: -1.
    private static final double TX_TO_TURRET_SIGN = -1.0;

    // Aimed when |tx| stays under this for AIMED_FRAMES fresh frames in a
    // row. TODO: TUNE once real shots show how much error a shot tolerates.
    private static final double AIM_TOLERANCE_DEG = 1.5;
    private static final int AIMED_FRAMES = 3;

    private final Turret turret = new Turret();
    private final CameraTilt tilt = new CameraTilt();

    private int alignedFrames = 0;
    private boolean usingVision = false;
    private double distanceIn = 0;

    public void init(HardwareMap hw) {
        turret.init(hw);
        tilt.init(hw);
    }

    public void update(Pose robotPose, Pose cell, HiveVision vision) {
        double dx = cell.getX() - robotPose.getX();
        double dy = cell.getY() - robotPose.getY();
        double odometryDistanceIn = FieldConstants.distanceToHive(robotPose, cell);
        double heightDiffIn = TAG_HEIGHT_ABOVE_TILES_IN - CAMERA_HEIGHT_ABOVE_TILES_IN;

        // Tilt always comes from odometry: a small range error barely moves
        // the tag in the frame, and a steady pitch keeps the ty math clean.
        tilt.setTargetAngleDeg(Math.toDegrees(Math.atan2(heightDiffIn, odometryDistanceIn)));

        usingVision = vision.isTargetVisible();
        if (usingVision) {
            double tx = vision.getTargetTxDeg();
            turret.setTargetAngleDeg(turret.getCurrentAngleDeg() + TX_TO_TURRET_SIGN * tx);
            alignedFrames = (Math.abs(tx) < AIM_TOLERANCE_DEG) ? alignedFrames + 1 : 0;

            double elevationRad = Math.toRadians(tilt.getTargetAngleDeg() + vision.getTargetTyDeg());
            distanceIn = (elevationRad > Math.toRadians(5))
                    ? heightDiffIn / Math.tan(elevationRad)
                    : odometryDistanceIn;
        } else {
            double bearingRad = Math.atan2(dy, dx);
            turret.setTargetAngleDeg(normalizeDeg(Math.toDegrees(bearingRad - robotPose.getHeading())));
            alignedFrames = 0;
            distanceIn = odometryDistanceIn;
        }
    }

    // Locked on by vision (tags centered for several frames).
    public boolean isAimedByVision() {
        return alignedFrames >= AIMED_FRAMES;
    }

    // Turret has reached the odometry-computed angle (used as a fallback
    // when the tags can't be seen).
    public boolean isAimedByOdometry() {
        return !usingVision && turret.isAtTarget();
    }

    public boolean isUsingVision() { return usingVision; }

    public double getDistanceIn() { return distanceIn; }

    public void stop() {
        turret.stop();
    }

    private static double normalizeDeg(double deg) {
        deg = deg % 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }
}
