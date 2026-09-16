package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Field-centric mecanum drivetrain: the IMU's yaw is used to rotate the
// driver's stick input into field coordinates each loop, so pushing the
// stick "forward" always drives toward the same field direction
// regardless of which way the robot is currently facing. Device names
// below must exactly match the names given to the four drive motors and
// the IMU in the Driver Hub's robot configuration.
//
// NOTE: field-centric driving is independent of per-wheel direction --
// it only rotates the (drive, strafe) vector before mixing. The
// setDirection() calls below are what was actually verified with
// systemTest/WheelDirectionTest (isolated single-wheel test); do not
// change them without re-running that test on this robot.
//
// Robot was rewired/renamed on the Driver Hub after the port table below
// was recorded -- it's now stale and kept only as history, not fact:
//   port 0: back_right_drive  (default FORWARD)
//   port 1: front_right_drive (REVERSE)
//   port 2: front_left_drive  (REVERSE)
//   port 3: back_left_drive   (REVERSE)
@TeleOp(name = "Basic TeleOp", group = "teleOp")
public class BasicTeleOp extends LinearOpMode {

    private static final String FRONT_LEFT_NAME = "front_left_drive";
    private static final String FRONT_RIGHT_NAME = "front_right_drive";
    private static final String BACK_LEFT_NAME = "back_left_drive";
    private static final String BACK_RIGHT_NAME = "back_right_drive";
    private static final String IMU_NAME = "imu";

    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.4;
    private static final double STRAFE_CORRECTION = 1.1;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        // Re-confirmed via systemTest/WheelDirectionTest after rewiring:
        // frontLeft/backLeft need REVERSE, frontRight/backRight stay at
        // default (FORWARD).
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // TODO: VERIFY -- logo/USB facing direction must match how the
        // Control/Expansion Hub is actually mounted on this robot, or
        // yaw will read backward/sideways. UP/BACKWARD is a placeholder.
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();

        telemetry.addLine("Ready. Field-centric: left stick = drive/strafe, right stick X = turn.");
        telemetry.addLine("D-pad up/down = full-speed forward/backward override. Right bumper = slow mode.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * STRAFE_CORRECTION;
            double turn = gamepad1.right_stick_x;

            if (gamepad1.dpad_up) forward = 1;
            else if (gamepad1.dpad_down) forward = -1;

            double speedScale = gamepad1.right_bumper ? SLOW_SPEED : NORMAL_SPEED;

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double robotAngle = Math.toRadians(angles.getYaw(AngleUnit.DEGREES));

            double rotatedForward = forward * Math.cos(robotAngle) + strafe * Math.sin(robotAngle);
            double rotatedStrafe = -forward * Math.sin(robotAngle) + strafe * Math.cos(robotAngle);

            double fl = rotatedForward + rotatedStrafe + turn;
            double fr = rotatedForward - rotatedStrafe - turn;
            double bl = rotatedForward - rotatedStrafe + turn;
            double br = rotatedForward + rotatedStrafe - turn;

            double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));

            frontLeft.setPower((fl / max) * speedScale);
            frontRight.setPower((fr / max) * speedScale);
            backLeft.setPower((bl / max) * speedScale);
            backRight.setPower((br / max) * speedScale);

            telemetry.addData("Yaw (deg)", "%.1f", angles.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Slow mode", gamepad1.right_bumper);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
