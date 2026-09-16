package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Bring-up test for the launcher motor -- no RPM tuning, no PIDF. Right
// trigger directly sets motor power live (0 = stopped, fully pressed =
// full power), so different power levels can be tried without redeploying
// code each time. Device name below must exactly match the name given to
// it in the Driver Hub's robot configuration (Configure Robot > Motors).
@TeleOp(name = "Launcher Raw Power Test", group = "systemTest")
public class LauncherRawPowerTest extends LinearOpMode {

    // Physically the launcher motor, but the Driver Hub config still has
    // it named front_right_drive (port 1 repurposed, no expansion hub).
    private static final String MOTOR_NAME = "front_right_drive";

    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready. Right trigger = power (live, no redeploy needed).");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = gamepad1.right_trigger;
            motor.setPower(power);

            telemetry.addData("Motor", MOTOR_NAME);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.update();
        }

        motor.setPower(0);
    }
}
