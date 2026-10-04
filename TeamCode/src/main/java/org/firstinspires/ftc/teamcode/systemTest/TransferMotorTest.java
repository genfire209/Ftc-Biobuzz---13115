package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

// Expansion Hub port 2: goBILDA 5202/3/4 series motor named "transfer".
// Spins at full power as soon as you press play. Change it live without rebuilding:
//   dpad up / down  power +/- 0.05
//   A               stop / start
//   B               reverse direction
@TeleOp(name = "Transfer Motor Test", group = "systemTest")
public class TransferMotorTest extends LinearOpMode {

    private static final String MOTOR_NAME = "transfer";
    private static final double START_POWER = 1.0;
    private static final double POWER_STEP = 0.05;

    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Press play to start " + MOTOR_NAME + ".");
        telemetry.update();

        waitForStart();

        double power = START_POWER;
        boolean running = true;
        boolean reversed = false;
        boolean prevUp = false, prevDown = false, prevA = false, prevB = false;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !prevUp) power = Range.clip(power + POWER_STEP, 0, 1);
            if (gamepad1.dpad_down && !prevDown) power = Range.clip(power - POWER_STEP, 0, 1);
            if (gamepad1.a && !prevA) running = !running;
            if (gamepad1.b && !prevB) reversed = !reversed;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevA = gamepad1.a;
            prevB = gamepad1.b;

            double output = running ? (reversed ? -power : power) : 0;
            motor.setPower(output);

            telemetry.addData("Motor", MOTOR_NAME);
            telemetry.addData("State", running ? (reversed ? "RUNNING (reversed)" : "RUNNING") : "STOPPED (A to start)");
            telemetry.addData("Power", "%.2f  (dpad up/down)", power);
            telemetry.addData("Output", "%.2f", output);
            telemetry.addLine("A = stop/start, B = reverse");
            telemetry.update();
        }

        motor.setPower(0);
    }
}
