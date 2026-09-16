package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Bring-up test for the launcher motor -- no RPM tuning, no PIDF, just
// raw power to confirm it spins and actually launches something. Device
// name below must exactly match the name given to it in the Driver Hub's
// robot configuration (Configure Robot > Motors).
@TeleOp(name = "Launcher Raw Power Test", group = "systemTest")
public class LauncherRawPowerTest extends LinearOpMode {

    // Physically the launcher motor, but the Driver Hub config still has
    // it named front_right_drive (port 1 repurposed, no expansion hub).
    private static final String MOTOR_NAME = "front_right_drive";
    private static final double MOTOR_POWER = 0.4;

    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready. Press play to start " + MOTOR_NAME + ".");
        telemetry.update();

        waitForStart();

        motor.setPower(MOTOR_POWER);

        while (opModeIsActive()) {
            telemetry.addData("Motor", MOTOR_NAME);
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.update();
        }

        motor.setPower(0);
    }
}
