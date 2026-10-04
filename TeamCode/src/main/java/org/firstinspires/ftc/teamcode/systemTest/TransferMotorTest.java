package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Expansion Hub port 2: goBILDA 5202/3/4 series motor named "transfer".
// Right joystick controls speed: push up = forward (all the way = full
// power), pull down = reverse, let go = stop.
@TeleOp(name = "Transfer Motor Test", group = "systemTest")
public class TransferMotorTest extends LinearOpMode {

    private static final String MOTOR_NAME = "transfer";

    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Press play, then move the RIGHT joystick up/down.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Joystick y is negative when pushed up, so flip it.
            double power = -gamepad1.right_stick_y;
            motor.setPower(power);

            telemetry.addData("Motor", MOTOR_NAME);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addLine("Right stick: up = forward, down = reverse");
            telemetry.update();
        }

        motor.setPower(0);
    }
}
