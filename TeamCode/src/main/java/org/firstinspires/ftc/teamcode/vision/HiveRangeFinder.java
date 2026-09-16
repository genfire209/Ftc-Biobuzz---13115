package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

// Reads live distance to the HIVE directly from its AprilTag cluster
// (mounted on the CELL itself, manual Section 9.9), so shot power is based
// on the actual physical target distance rather than an assumed field
// coordinate -- immune to the manual's stated +/-1in field-to-field
// tolerance and to Pinpoint drift. This is the primary distance source;
// HiveShootAutoBase falls back to odometry-based
// FieldConstants.distanceToHive() whenever no cluster is visible that
// cycle.
//
// This season's SDK (12.0.0) detects the HIVE's 4-tag stickers as a single
// fused AprilTagClusterDetection (not 4 separate single-tag detections),
// via AprilTagGameDatabase's built-in BIOBUZZ tag library, identified by
// name -- "RED SCORING" / "BLUE SCORING" -- rather than by raw tag ID.
public class HiveRangeFinder {

    // TODO: DECIDE -- webcam device name, must match Driver Hub config.
    private static final String WEBCAM_NAME = "webcam";

    private static final String RED_SCORING_CLUSTER_NAME = "RED SCORING";
    private static final String BLUE_SCORING_CLUSTER_NAME = "BLUE SCORING";

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private String targetClusterName;

    public void init(HardwareMap hw, FieldConstants.Alliance alliance) {
        targetClusterName = (alliance == FieldConstants.Alliance.RED)
                ? RED_SCORING_CLUSTER_NAME
                : BLUE_SCORING_CLUSTER_NAME;

        // easyCreateWithDefaults() pulls in AprilTagGameDatabase's current
        // (BIOBUZZ) tag library automatically, which already defines the
        // RED SCORING / BLUE SCORING clusters.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hw.get(WebcamName.class, WEBCAM_NAME), aprilTag);
    }

    // Live distance (inches) to the HIVE's own "<alliance> SCORING"
    // AprilTag cluster, or null if it isn't visible this cycle.
    public Double getRangeIn() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;

            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata == null) continue;
            if (!targetClusterName.equals(cluster.metadata.shortName)) continue;

            return cluster.ftcPose.range;
        }

        return null;
    }

    public void close() {
        if (visionPortal != null) visionPortal.close();
    }
}
