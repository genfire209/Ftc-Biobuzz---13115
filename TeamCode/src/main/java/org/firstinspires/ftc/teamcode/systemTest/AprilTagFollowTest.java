package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

// Bring-up test, not the competition auto: if the Limelight sees ANY
// AprilTag, drive toward it (proportional steering to keep it centered,
// proportional braking as it approaches STOP_DISTANCE_IN) and hold once
// close. No tag visible for longer than TAG_LOSS_GRACE_S -> robot holds
// still. Runs as a TeleOp (not @Autonomous) so STOP is always one button
// away and there's no 30s timeout while testing.
@TeleOp(name = "AprilTag Follow Test", group = "systemTest")
public class AprilTagFollowTest extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "Limelight-13115";
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    private static final double TEST_POWER = 0.7;
    private static final double STOP_DISTANCE_IN = 12.0;
    private static final double DRIVE_KP = 0.05; // power per inch of remaining distance

    // TODO: VERIFY -- sign depends on which way the Limelight is
    // physically mounted (right-side-up vs. rotated/flipped). If the
    // robot turns AWAY from a centered tag instead of toward it, that's
    // an inverted sign here -- flip it. Confirmed working sign is
    // negative for this robot's mount.
    private static final double TURN_KP = -0.03; // power per degree of horizontal error

    // Don't slam to a dead stop on a single dropped detection frame --
    // hold the last commanded drive/turn briefly, since camera shake
    // from the robot's own motion can cause momentary tag-loss that has
    // nothing to do with the tag actually being gone.
    private static final double TAG_LOSS_GRACE_S = 0.3;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
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
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        telemetry.addLine("Ready. Robot follows any visible AprilTag, up to " + TEST_POWER + " power.");
        telemetry.update();

        waitForStart();

        double lastDrive = 0;
        double lastTurn = 0;
        double timeSinceSeenS = 0;
        long lastLoopNs = System.nanoTime();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastLoopNs) / 1e9;
            lastLoopNs = now;

            LLResult result = limelight.getLatestResult();
            boolean seesTag = false;
            double drive = 0;
            double turn = 0;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult tag = fiducials.get(0);
                    seesTag = true;
                    timeSinceSeenS = 0;

                    double tx = tag.getTargetXDegrees();
                    Position pos = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                    double distanceIn = Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);

                    turn = Range.clip(tx * TURN_KP, -TEST_POWER, TEST_POWER);
                    // Proportional braking: full power far away, smoothly
                    // decelerating to 0 by STOP_DISTANCE_IN instead of an
                    // abrupt full-power-then-zero cutoff.
                    drive = Range.clip((distanceIn - STOP_DISTANCE_IN) * DRIVE_KP, 0, TEST_POWER);

                    lastDrive = drive;
                    lastTurn = turn;

                    telemetry.addData("Tag ID", tag.getFiducialId());
                    telemetry.addData("Distance (in)", "%.1f", distanceIn);
                    telemetry.addData("Tx (deg)", "%.1f", tx);
                }
            }

            if (!seesTag) {
                timeSinceSeenS += dt;
                if (timeSinceSeenS < TAG_LOSS_GRACE_S) {
                    // Brief dropout -- keep coasting on the last known
                    // command instead of stopping dead.
                    drive = lastDrive;
                    turn = lastTurn;
                } else {
                    lastDrive = 0;
                    lastTurn = 0;
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

            telemetry.addData("Tag visible", seesTag);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        limelight.stop();
    }
}
