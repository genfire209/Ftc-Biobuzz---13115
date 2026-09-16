package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Launcher;

// Bench-test the Launcher's ramped-RPM control in isolation, before wiring
// it into autonomous. Put the robot on blocks (wheels off the ground isn't
// relevant here, but keep clear of the flywheel). A/B/X/Y set fixed target
// RPMs; watch telemetry for target vs. measured RPM converging smoothly
// without oscillation, and confirm isReadyToFire() doesn't trigger early
// on noisy encoder readings.
@TeleOp(name = "Launcher Bench Test", group = "systemTest")
public class LauncherBenchTest extends LinearOpMode {

    // TODO: TUNE -- pick RPM presets that make sense once the launcher's
    // real free-speed/safe-RPM range is known.
    private static final double RPM_A = 1000;
    private static final double RPM_B = 2000;
    private static final double RPM_X = 3000;
    private static final double RPM_Y = 4000;

    private final Launcher launcher = new Launcher();

    @Override
    public void runOpMode() throws InterruptedException {
        launcher.init(hardwareMap);

        telemetry.addLine("Ready. A/B/X/Y = RPM presets, right bumper = stop.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) launcher.setTargetRpm(RPM_A);
            else if (gamepad1.b) launcher.setTargetRpm(RPM_B);
            else if (gamepad1.x) launcher.setTargetRpm(RPM_X);
            else if (gamepad1.y) launcher.setTargetRpm(RPM_Y);
            else if (gamepad1.right_bumper) launcher.setTargetRpm(0);

            launcher.update();

            telemetry.addData("Measured RPM", "%.0f", launcher.getMeasuredRpm());
            telemetry.addData("Ready to fire", launcher.isReadyToFire());
            telemetry.update();
        }

        launcher.stop();
    }
}
