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
// AprilTag, drive toward it (simple proportional steering to keep it
// centered, forward power capped at TEST_POWER) and stop once within
// STOP_DISTANCE_IN. No tag visible -> robot holds still. Runs as a
// TeleOp (not @Autonomous) so STOP is always one button away and there's
// no 30s timeout while testing.
@TeleOp(name = "AprilTag Follow Test", group = "systemTest")
public class AprilTagFollowTest extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "Limelight-13115";
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    private static final double TEST_POWER = 0.7;
    private static final double STOP_DISTANCE_IN = 12.0;
    private static final double TURN_KP = 0.03; // power per degree of horizontal error

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

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double drive = 0;
            double turn = 0;
            boolean seesTag = false;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult tag = fiducials.get(0);
                    seesTag = true;

                    double tx = tag.getTargetXDegrees();
                    Position pos = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                    double distanceIn = Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);

                    turn = Range.clip(tx * TURN_KP, -TEST_POWER, TEST_POWER);
                    drive = (distanceIn > STOP_DISTANCE_IN) ? TEST_POWER : 0;

                    telemetry.addData("Tag ID", tag.getFiducialId());
                    telemetry.addData("Distance (in)", "%.1f", distanceIn);
                    telemetry.addData("Tx (deg)", "%.1f", tx);
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
