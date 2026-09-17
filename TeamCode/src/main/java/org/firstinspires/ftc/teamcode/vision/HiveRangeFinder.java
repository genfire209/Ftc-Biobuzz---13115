package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Reads live distance to the HIVE directly from its AprilTag cluster
// (mounted on the CELL itself, manual Section 9.9), so shot power is based
// on the actual physical target distance rather than an assumed field
// coordinate -- immune to the manual's stated +/-1in field-to-field
// tolerance and to Pinpoint drift. This is the primary distance source;
// HiveShootAutoBase falls back to odometry-based
// FieldConstants.distanceToHive() whenever no target tag is visible that
// cycle.
//
// Limelight does its own on-device AprilTag detection and reports raw
// per-tag fiducial results (unlike a webcam through the FTC SDK's
// VisionPortal/AprilTagProcessor, which fuses the HIVE's 4-tag sticker
// into one AprilTagClusterDetection) -- so the HIVE's scoring cluster is
// identified here by its member tag IDs directly (manual Figure 9-17):
// 30-33 = RED SCORING, 42-45 = BLUE SCORING.
//
// VISIBILITY: the manual states each cluster is mounted "on the bottom
// of a CELL facing downward towards the TILES" (Section 9.9), not facing
// outward toward the field like last season's goal tags -- a fixed-mount
// camera in a normal shooting position likely can't see it. The team is
// building a pan/tilt mount for the Limelight to address this (aim the
// camera up under the CELL to find the tag). Note this doesn't require
// any change to the range math here: getRangeIn()'s 3D distance
// (sqrt(x^2+y^2+z^2) in camera space) is orientation-independent --
// panning/tilting only affects whether the tag is in frame, not the
// computed range once it's detected. See hardware/PanTilt.java (once
// built) for the aiming control; this class stays a pure "read whatever
// the camera currently sees" reader.
//
// Also note: the tag is physically offset from the actual scoring
// opening (which is higher up, at the cell entrance) -- getRangeIn()
// returns raw range to the tag itself, not a corrected distance to the
// entry point. Left uncorrected for now since the offset is small
// relative to typical shot distance; revisit if shots consistently
// under/over-shoot near the tag-measurement boundary.
public class HiveRangeFinder {

    private static final String LIMELIGHT_NAME = "Limelight-13115";

    // TODO: DECIDE -- which pipeline slot on the Limelight (configured via
    // its own web UI) has AprilTag detection set up, if not 0.
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    private static final int[] RED_SCORING_TAG_IDS = {30, 31, 32, 33};
    private static final int[] BLUE_SCORING_TAG_IDS = {42, 43, 44, 45};

    private Limelight3A limelight;
    private int[] targetIds;

    public void init(HardwareMap hw, FieldConstants.Alliance alliance) {
        targetIds = (alliance == FieldConstants.Alliance.RED) ? RED_SCORING_TAG_IDS : BLUE_SCORING_TAG_IDS;

        limelight = hw.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();
    }

    // Live distance (inches) to the HIVE's scoring tags, averaged across
    // any matching tag IDs detected this cycle (reduces per-tag pose
    // noise), or null if none of the target IDs are visible right now.
    public Double getRangeIn() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        double sum = 0.0;
        int count = 0;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (!isTargetId(fiducial.getFiducialId())) continue;

            Position pos = fiducial.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            sum += Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
            count++;
        }

        return (count == 0) ? null : sum / count;
    }

    public void close() {
        if (limelight != null) limelight.stop();
    }

    private boolean isTargetId(int id) {
        for (int targetId : targetIds) {
            if (targetId == id) return true;
        }
        return false;
    }
}
