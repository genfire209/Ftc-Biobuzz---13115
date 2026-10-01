package org.firstinspires.ftc.teamcode.systemTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

// Bring-up test, not the competition auto. HOLD RIGHT BUMPER and the robot
// turns toward the AprilTag and drives up to it, slowing down as the tag
// gets bigger in the image and stopping at STOP_TA. Let go of the bumper to
// stop instantly.
//
// Controller pattern copied from a BIOBUZZ pollen-chase that tracks smoothly
// (SASApantheon5193/biobuzz-pollen-vision): low power caps, distance from
// target AREA (ta) instead of the jumpy 3D pose, turn-first when far off
// center, and only acting on fresh camera frames.
//
// Live tuning on gamepad1 (no rebuild needed):
//   dpad left/right  turn kP -/+        dpad down/up  turn kD -/+
//   B                cycle step size    Y / A         stop area +/- 0.5%
//   X                flip turn direction (if it turns AWAY from the tag)
@TeleOp(name = "AprilTag Follow Test", group = "systemTest")
public class AprilTagFollowTest extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "Limelight-13115";
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    private static final double MAX_DRIVE = 0.35;
    private static final double MAX_TURN = 0.30;

    // ta is percent of the image (0-100) from LLResult.getTa(). A 3.25in
    // BIOBUZZ tag is roughly 4.5% at ~18in and 0.7% at ~45in on a
    // Limelight 3A. TODO: MEASURE -- tune STOP_TA live with Y/A.
    private static final double DEFAULT_STOP_TA = 4.5;
    private static final double FAR_TA_RATIO = 0.15; // full MAX_DRIVE below STOP_TA * this

    private static final double DEFAULT_TURN_KP = 0.02; // power per degree of tx
    private static final double DEFAULT_TURN_KD = 0.0001;
    // Negative was the confirmed-working sign on this robot's mount.
    private static final double DEFAULT_TURN_SIGN = -1.0;
    private static final double TX_DEADBAND_DEG = 1.5;
    // When the tag is this far off center, mostly turn and only creep forward.
    private static final double TURN_FIRST_DEG = 15.0;
    private static final double TURN_FIRST_DRIVE_SCALE = 0.3;

    // Ignore camera frames older than this.
    private static final long MAX_STALENESS_MS = 200;
    // Keep steering at the last seen tag this long after losing it, then stop.
    private static final double TAG_LOSS_GRACE_S = 0.5;

    private static final double[] STEP_SIZES = {0.01, 0.001, 0.0001};

    private static final String LOG_TAG = "AprilTagFollow";
    private static final double LOG_INTERVAL_S = 0.1;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Show telemetry on the Driver Station and the browser dashboard.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Same directions confirmed correct via WheelDirectionTest.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        telemetry.addLine("Ready. HOLD RIGHT BUMPER to follow the AprilTag.");
        telemetry.update();

        waitForStart();

        double turnKp = DEFAULT_TURN_KP;
        double turnKd = DEFAULT_TURN_KD;
        double turnSign = DEFAULT_TURN_SIGN;
        double stopTa = DEFAULT_STOP_TA;
        int stepIndex = 1;

        boolean prevLeft = false, prevRight = false, prevUp = false, prevDown = false;
        boolean prevB = false, prevY = false, prevA = false, prevX = false;

        boolean everSeen = false;
        int lastId = -1;
        double lastTx = 0, lastTa = 0;
        double lastError = 0;
        boolean haveLastError = false;
        double timeSinceSeenS = 0;
        double timeSinceLogS = 0;
        long lastLoopNs = System.nanoTime();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastLoopNs) / 1e9;
            lastLoopNs = now;

            // ---- Live tuning (rising edges only) ----
            double step = STEP_SIZES[stepIndex];
            if (gamepad1.dpad_right && !prevRight) turnKp += step;
            if (gamepad1.dpad_left && !prevLeft) turnKp = Math.max(0, turnKp - step);
            if (gamepad1.dpad_up && !prevUp) turnKd += step;
            if (gamepad1.dpad_down && !prevDown) turnKd = Math.max(0, turnKd - step);
            if (gamepad1.b && !prevB) stepIndex = (stepIndex + 1) % STEP_SIZES.length;
            if (gamepad1.y && !prevY) stopTa += 0.5;
            if (gamepad1.a && !prevA) stopTa = Math.max(0.5, stopTa - 0.5);
            if (gamepad1.x && !prevX) turnSign = -turnSign;
            prevRight = gamepad1.dpad_right; prevLeft = gamepad1.dpad_left;
            prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
            prevB = gamepad1.b; prevY = gamepad1.y; prevA = gamepad1.a; prevX = gamepad1.x;

            // ---- Vision: only fresh frames that actually contain a tag ----
            LLResult result = limelight.getLatestResult();
            boolean seesTag = false;
            if (result != null && result.getStaleness() < MAX_STALENESS_MS) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    seesTag = true;
                    everSeen = true;
                    timeSinceSeenS = 0;
                    lastId = fiducials.get(0).getFiducialId();
                    // Primary target (pipeline sorts Largest, Single Target).
                    lastTx = result.getTx();
                    lastTa = result.getTa();
                }
            }
            if (!seesTag) {
                timeSinceSeenS += dt;
            }
            boolean haveTarget = everSeen && timeSinceSeenS < TAG_LOSS_GRACE_S;

            // ---- Decide drive / turn ----
            double drive = 0, turn = 0;
            String mode;
            if (!gamepad1.right_bumper) {
                mode = "IDLE (hold right bumper)";
                haveLastError = false;
            } else if (!haveTarget) {
                mode = "NO TAG";
                haveLastError = false;
            } else if (lastTa >= stopTa) {
                mode = "ARRIVED";
                haveLastError = false;
            } else {
                mode = seesTag ? "FOLLOW" : "FOLLOW (coasting)";

                // PD turn on tx, with a deadband so it doesn't wiggle when centered.
                double error = lastTx;
                double dTerm = 0;
                if (seesTag && haveLastError && dt > 0) {
                    dTerm = (error - lastError) / dt * turnKd;
                }
                if (seesTag) {
                    lastError = error;
                    haveLastError = true;
                }
                if (Math.abs(error) > TX_DEADBAND_DEG) {
                    turn = Range.clip(turnSign * (error * turnKp + dTerm), -MAX_TURN, MAX_TURN);
                }

                // Full speed while the tag is small, ramping to 0 at stopTa.
                double farTa = stopTa * FAR_TA_RATIO;
                double t = Range.clip((lastTa - farTa) / (stopTa - farTa), 0, 1);
                drive = MAX_DRIVE * (1.0 - t);
                if (Math.abs(error) > TURN_FIRST_DEG) {
                    drive *= TURN_FIRST_DRIVE_SCALE;
                }
            }

            double fl = drive + turn;
            double fr = drive - turn;
            double bl = drive + turn;
            double br = drive - turn;

            double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));

            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            timeSinceLogS += dt;
            if (timeSinceLogS >= LOG_INTERVAL_S) {
                timeSinceLogS = 0;
                RobotLog.dd(LOG_TAG, "mode=%s seesTag=%b id=%d tx=%.2f ta=%.3f turn=%.3f drive=%.3f kP=%.4f kD=%.5f sign=%.0f stopTa=%.1f",
                        mode, seesTag, lastId, lastTx, lastTa, turn, drive, turnKp, turnKd, turnSign, stopTa);
            }

            telemetry.addData("Mode", mode);
            telemetry.addData("Tag", seesTag ? ("ID " + lastId) : "not visible");
            telemetry.addData("tx (deg)", "%.1f", lastTx);
            telemetry.addData("ta (%)", "%.2f  (stop at %.1f)", lastTa, stopTa);
            telemetry.addData("drive / turn", "%.2f / %.2f", drive, turn);
            telemetry.addLine("--- tuning ---");
            telemetry.addData("Turn kP", "%.4f (dpad L/R)", turnKp);
            telemetry.addData("Turn kD", "%.5f (dpad U/D)", turnKd);
            telemetry.addData("Step", "%.4f (B)", STEP_SIZES[stepIndex]);
            telemetry.addData("Turn sign", "%.0f (X flips)", turnSign);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        limelight.stop();
    }
}
