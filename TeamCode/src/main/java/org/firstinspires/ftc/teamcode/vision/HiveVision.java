package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;
import java.util.List;

// Reads the HIVE's AprilTag clusters from the Limelight. Each CELL has a
// 4-tag cluster on its bottom face, pointing at the floor (manual 9.6).
//
// Call update() once per loop, then read tx/ty for whichever CELL is the
// current target (setTargetTags). Distance comes from ty and the known
// camera/tag heights in HiveAimer, not from the Limelight's 3D pose --
// the 3D pose is the jumpiest reading when the tag is seen this steeply.
//
// Like systemTest/AprilTagFollowTest, this checks the fiducial list
// directly instead of gating on result.isValid() (whose flag didn't
// behave as expected in testing -- see commit ae9cc92) and ignores frames
// older than MAX_STALENESS_MS.
public class HiveVision {

    private static final String LIMELIGHT_NAME = "Limelight-13115";

    // Pipeline 0 = AprilTag pipeline tuned 2026-09-30/10-02 (exposure 350,
    // gain 15, 640x480, downscale 2, marker size 82.55mm).
    private static final int APRILTAG_PIPELINE_INDEX = 0;
    private static final long MAX_STALENESS_MS = 200;

    private Limelight3A limelight;
    private LLResult latest;
    private int[] targetTags = new int[0];

    private boolean targetVisible;
    private double targetTxDeg;
    private double targetTyDeg;

    public void init(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();
    }

    public void setTargetTags(int[] tags) {
        targetTags = tags;
    }

    // Grabs the newest frame and averages tx/ty over the target CELL's tags
    // seen in it (averaging the 4-tag cluster steadies the reading).
    public void update() {
        LLResult result = limelight.getLatestResult();
        latest = (result != null && result.getStaleness() < MAX_STALENESS_MS) ? result : null;

        double sumTx = 0, sumTy = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult f : fiducials()) {
            if (!contains(targetTags, f.getFiducialId())) continue;
            sumTx += f.getTargetXDegrees();
            sumTy += f.getTargetYDegrees();
            count++;
        }
        targetVisible = count > 0;
        targetTxDeg = targetVisible ? sumTx / count : 0;
        targetTyDeg = targetVisible ? sumTy / count : 0;
    }

    public boolean isTargetVisible() { return targetVisible; }

    // Degrees; positive = tag is right of the crosshair.
    public double getTargetTxDeg() { return targetTxDeg; }

    // Degrees; positive = tag is above the crosshair.
    public double getTargetTyDeg() { return targetTyDeg; }

    // True if any of the given tags is in the current frame (used to spot
    // the other CELL coming up after a TIP).
    public boolean seesAny(int[] tags) {
        for (LLResultTypes.FiducialResult f : fiducials()) {
            if (contains(tags, f.getFiducialId())) return true;
        }
        return false;
    }

    public void close() {
        if (limelight != null) limelight.stop();
    }

    private List<LLResultTypes.FiducialResult> fiducials() {
        List<LLResultTypes.FiducialResult> list = (latest == null) ? null : latest.getFiducialResults();
        return (list == null) ? Collections.<LLResultTypes.FiducialResult>emptyList() : list;
    }

    private static boolean contains(int[] ids, int id) {
        for (int x : ids) if (x == id) return true;
        return false;
    }
}
