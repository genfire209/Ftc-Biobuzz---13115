package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Drives exactly one drive motor at a time, so each wheel's true rotation
// direction can be confirmed with zero ambiguity from mecanum mixing math.
// Use this instead of BasicTeleOp when diagnosing "which wheel is
// reversed" -- during normal driving, diagonal wheel pairs (frontLeft/
// backRight vs. frontRight/backLeft) legitimately spin opposite
// directions while strafing, which is easy to mistake for a bug.
//
// Y = frontLeft, B = frontRight, X = backLeft, A = backRight (Xbox/
// Logitech-in-XInput-mode layout). Hold a button: that one wheel spins
// forward at TEST_POWER. Release: it stops. Watch each wheel from above
// and confirm it turns the direction that would drive the robot forward.
@TeleOp(name = "Wheel Direction Test", group = "systemTest")
public class WheelDirectionTest extends LinearOpMode {

    private static final double TEST_POWER = 0.3;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Apply the SAME direction settings as BasicTeleOp, so this test
        // reflects what driving will actually do -- if a wheel still
        // looks wrong here, fix it in both files together.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Y=frontLeft  B=frontRight  X=backLeft  A=backRight");
        telemetry.addLine("Hold a button. That wheel should spin the direction");
        telemetry.addLine("that drives the robot FORWARD.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            frontLeft.setPower(gamepad1.y ? TEST_POWER : 0);
            frontRight.setPower(gamepad1.b ? TEST_POWER : 0);
            backLeft.setPower(gamepad1.x ? TEST_POWER : 0);
            backRight.setPower(gamepad1.a ? TEST_POWER : 0);

            telemetry.addData("frontLeft (Y)", gamepad1.y ? "SPINNING" : "stopped");
            telemetry.addData("frontRight (B)", gamepad1.b ? "SPINNING" : "stopped");
            telemetry.addData("backLeft (X)", gamepad1.x ? "SPINNING" : "stopped");
            telemetry.addData("backRight (A)", gamepad1.a ? "SPINNING" : "stopped");
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
