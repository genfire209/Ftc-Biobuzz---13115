package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Expansion Hub, motor port 0: 12V Matrix motor named "launcher_pollen".
// Expansion Hub, motor port 2: goBILDA 6000 RPM motor named "transfer".
//
//   Right joystick  launcher_pollen speed, full range: up = forward,
//                   down = reverse, let go = stop.
//   The transfer runs at TRANSFER_POWER in the same direction as the
//   launcher whenever the launcher is running, and stops when it stops.
@TeleOp(name = "Transfer + Launcher Test", group = "systemTest")
public class TransferLauncherTest extends LinearOpMode {

    private static final String LAUNCHER_NAME = "launcher_pollen";
    private static final String TRANSFER_NAME = "transfer";

    // Team-confirmed best transfer speed (6000 RPM motor).
    private static final double TRANSFER_POWER = 0.5;
    // Stick has to move past this before the launcher counts as running,
    // so a slightly off-center stick doesn't start the transfer.
    private static final double STICK_DEADBAND = 0.05;

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

        telemetry.addLine("Ready. Right stick runs the launcher; transfer follows it.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Joystick y is negative when pushed up, so flip it.
            double stick = -gamepad1.right_stick_y;
            boolean launcherOn = Math.abs(stick) > STICK_DEADBAND;

            double launcherPower = launcherOn ? stick : 0;
            // Same direction as the launcher: forward with it, reverse with it.
            double transferPower = launcherOn ? Math.signum(stick) * TRANSFER_POWER : 0;

            launcher.setPower(launcherPower);
            transfer.setPower(transferPower);

            telemetry.addData("Launcher power", "%.2f  (right stick)", launcherPower);
            telemetry.addData("Transfer power", "%.2f  (%s)", transferPower, launcherOn ? (stick > 0 ? "forward with launcher" : "reverse with launcher") : "off");
            telemetry.update();
        }

        launcher.setPower(0);
        transfer.setPower(0);
    }
}
