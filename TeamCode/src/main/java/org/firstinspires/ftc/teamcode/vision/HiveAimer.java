package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CameraTilt;
import org.firstinspires.ftc.teamcode.hardware.Turret;

// Points the Limelight (turret yaw + tilt pitch) at the HIVE, computed
// from Pedro's live robot pose and the known/assumed HIVE target
// location, rather than a fixed angle or a blind sweep/search (which
// would eat into AUTO's 30s window). Call update() every loop, same
// non-blocking pattern as Launcher/Follower -- it issues a new target
// angle each cycle and returns immediately.
public class HiveAimer {

    // TODO: MEASURE -- approximate height of the AprilTag cluster itself
    // (mounted on the bottom face of the CELL, manual Section 9.9) above
    // the tiles -- NOT the same as the cell opening height
    // (FieldConstants.CELL_BOTTOM_OPENING_ABOVE_TILES_IN). Rough estimate
    // only; refine once the physical HIVE/CAD is available.
    private static final double TAG_HEIGHT_ABOVE_TILES_IN = 30.0;

    // TODO: MEASURE -- height of the Limelight's tilt pivot above the
    // tiles, once the turret/tilt mount is built.
    private static final double CAMERA_HEIGHT_ABOVE_TILES_IN = 12.0;

    private final Turret turret = new Turret();
    private final CameraTilt tilt = new CameraTilt();

    public void init(HardwareMap hw) {
        turret.init(hw);
        tilt.init(hw);
    }

    // Aims at hiveTarget given the robot's current field pose (from
    // follower.getPose()). Non-blocking.
    public void update(Pose robotPose, Pose hiveTarget) {
        double dx = hiveTarget.getX() - robotPose.getX();
        double dy = hiveTarget.getY() - robotPose.getY();
        double horizontalDistanceIn = Math.hypot(dx, dy);

        double fieldBearingRad = Math.atan2(dy, dx);
        double turretAngleDeg = normalizeDeg(Math.toDegrees(fieldBearingRad - robotPose.getHeading()));

        double heightDiffIn = TAG_HEIGHT_ABOVE_TILES_IN - CAMERA_HEIGHT_ABOVE_TILES_IN;
        double tiltAngleDeg = Math.toDegrees(Math.atan2(heightDiffIn, horizontalDistanceIn));

        turret.setTargetAngleDeg(turretAngleDeg);
        tilt.setTargetAngleDeg(tiltAngleDeg);
    }

    public boolean isAimed() {
        return turret.isAtTarget();
    }

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
