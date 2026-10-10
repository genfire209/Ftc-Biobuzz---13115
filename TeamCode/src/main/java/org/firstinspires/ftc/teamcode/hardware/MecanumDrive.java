package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Robot-centric mecanum drive for the meet-1 robot (belt drive, four
// goBILDA 312 RPM motors on Control Hub motor ports 0-3). Used by both
// MeetOneTeleOp and MeetOneAutoBase.
//
// Every wheel must turn the way that drives the robot toward the intake for
// drive(1, 0, 0). On the assembled robot (2026-10-10) the drive motors are
// plugged in left/right mirrored from the original plan: Control Hub port
// 0 = front RIGHT, 1 = front LEFT, 2 = back RIGHT, 3 = back LEFT. The hub
// config names match that wiring, so these directions are per real wheel
// (left side REVERSE, right side FORWARD).
public class MecanumDrive {

    public static final String FRONT_LEFT_NAME = "front_left_drive";
    public static final String FRONT_RIGHT_NAME = "front_right_drive";
    public static final String BACK_LEFT_NAME = "back_left_drive";
    public static final String BACK_RIGHT_NAME = "back_right_drive";

    public static final DcMotor.Direction FRONT_LEFT_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction FRONT_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction BACK_LEFT_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction BACK_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public void init(HardwareMap hw) {
        frontLeft = hw.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = hw.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft = hw.get(DcMotor.class, BACK_LEFT_NAME);
        backRight = hw.get(DcMotor.class, BACK_RIGHT_NAME);

        frontLeft.setDirection(FRONT_LEFT_DIRECTION);
        frontRight.setDirection(FRONT_RIGHT_DIRECTION);
        backLeft.setDirection(BACK_LEFT_DIRECTION);
        backRight.setDirection(BACK_RIGHT_DIRECTION);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // forward: + = toward the intake (the robot's front).
    // strafeRight: + = sideways to the robot's right.
    // turnClockwise: + = clockwise seen from above (right stick pushed right).
    // Scaled down together if any wheel would go past full power.
    public void drive(double forward, double strafeRight, double turnClockwise) {
        double fl = forward + strafeRight + turnClockwise;
        double fr = forward - strafeRight - turnClockwise;
        double bl = forward - strafeRight + turnClockwise;
        double br = forward + strafeRight - turnClockwise;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    public void stop() {
        drive(0, 0, 0);
    }
}
