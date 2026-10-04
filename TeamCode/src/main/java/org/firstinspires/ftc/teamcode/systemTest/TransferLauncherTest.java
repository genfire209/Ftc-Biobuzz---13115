package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Expansion Hub, motor port 0: 12V Matrix motor named "launcher_pollen".
// Expansion Hub, motor port 2: goBILDA 6000 RPM motor named "transfer".
//
//   Right joystick  launcher_pollen speed: up = forward, down = reverse,
//                   let go = stop. Capped at MAX_POWER.
//   Hold A (Cross)  run the transfer at TRANSFER_POWER.
@TeleOp(name = "Transfer + Launcher Test", group = "systemTest")
public class TransferLauncherTest extends LinearOpMode {

    private static final String LAUNCHER_NAME = "launcher_pollen";
    private static final String TRANSFER_NAME = "transfer";

    // Neither motor is ever driven above 50% power.
    private static final double MAX_POWER = 0.5;
    // Team-confirmed best transfer speed (6000 RPM motor).
    private static final double TRANSFER_POWER = 0.5;

    private DcMotor launcher;
    private DcMotor transfer;

    @Override
    public void runOpMode() throws InterruptedException {
        launcher = hardwareMap.get(DcMotor.class, LAUNCHER_NAME);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER_NAME);

        for (DcMotor motor : new DcMotor[]{launcher, transfer}) {
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Let the launcher wheel coast down instead of slamming to a stop.
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready. Right stick = launcher, hold A (Cross) = transfer.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Joystick y is negative when pushed up, so flip it.
            double launcherPower = -gamepad1.right_stick_y * MAX_POWER;
            double transferPower = gamepad1.a ? Math.min(TRANSFER_POWER, MAX_POWER) : 0;

            launcher.setPower(launcherPower);
            transfer.setPower(transferPower);

            telemetry.addData("Launcher power", "%.2f  (right stick, max %.2f)", launcherPower, MAX_POWER);
            telemetry.addData("Transfer power", "%.2f  (hold A / Cross)", transferPower);
            telemetry.update();
        }

        launcher.setPower(0);
        transfer.setPower(0);
    }
}
