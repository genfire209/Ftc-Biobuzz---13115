package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Tests one wheel on Control Hub motor port 0, which the robot config
// names "front_left_drive".
//   Left joystick up/down  wheel speed: up = forward, down = reverse,
//                          let go = stop (full stick = full power).
@TeleOp(name = "Single Wheel Test", group = "systemTest")
public class SingleWheelTest extends LinearOpMode {

    private static final String MOTOR_NAME = "front_left_drive";

    private DcMotor wheel;

    @Override
    public void runOpMode() throws InterruptedException {
        wheel = hardwareMap.get(DcMotor.class, MOTOR_NAME);

        wheel.setDirection(DcMotor.Direction.FORWARD);
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Press play, then move the LEFT joystick up/down.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Joystick y is negative when pushed up, so flip it.
            double power = -gamepad1.left_stick_y;
            wheel.setPower(power);

            telemetry.addData("Wheel", "%s (Control Hub port 0)", MOTOR_NAME);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Encoder", wheel.getCurrentPosition());
            telemetry.update();
        }

        wheel.setPower(0);
    }
}
