package org.firstinspires.ftc.teamcode.systemTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Bench test for the goBILDA Pinpoint + two SWYFT Linear Odometry pods,
// before anything is mounted. Wiring: Pinpoint -> Control Hub I2C bus 1
// (config name "pinpoint"); forward pod -> Pinpoint X port, strafe pod -> Y port.
//
// 1. Status should read READY (Pinpoint LED green). Keep it still at INIT.
// 2. Roll each pod's wheel by hand: its raw count must change, the other's not.
// 3. Calibrate: press A to zero, roll ONE pod exactly 200 mm along a ruler,
//    read "ticks per mm". ~34 means 4096 counts/rev; ~137 means 16384.
// 4. Turn the Pinpoint flat on the table one full circle: heading ~360 deg.
//   A = zero the counts    B = reset position + recalibrate IMU (hold still)
@TeleOp(name = "Pinpoint Bench Test", group = "systemTest")
public class PinpointBenchTest extends LinearOpMode {

    private static final String PINPOINT_NAME = "pinpoint";
    private static final double CAL_DISTANCE_MM = 200.0;
    private static final double SWYFT_WHEEL_DIAMETER_MM = 38.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Show telemetry on the Driver Station and the browser dashboard.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        // Placeholder resolution (4096 counts per 38 mm wheel turn) only so the
        // mm readout is roughly sensible; the raw counts below are what matter.
        pinpoint.setEncoderResolution(4096.0 / (Math.PI * SWYFT_WHEEL_DIAMETER_MM), DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Keep the Pinpoint still. Press START when status is READY.");
        telemetry.update();
        while (opModeInInit()) {
            pinpoint.update();
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addData("Device version", pinpoint.getDeviceVersion());
            telemetry.update();
        }

        int zeroX = pinpoint.getEncoderX();
        int zeroY = pinpoint.getEncoderY();
        boolean prevA = false, prevB = false;

        while (opModeIsActive()) {
            pinpoint.update();
            if (gamepad1.a && !prevA) {
                zeroX = pinpoint.getEncoderX();
                zeroY = pinpoint.getEncoderY();
            }
            if (gamepad1.b && !prevB) {
                pinpoint.resetPosAndIMU();
            }
            prevA = gamepad1.a;
            prevB = gamepad1.b;

            int dx = pinpoint.getEncoderX() - zeroX;
            int dy = pinpoint.getEncoderY() - zeroY;

            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addData("Pinpoint loop", "%d us (%.0f Hz)", pinpoint.getLoopTime(), pinpoint.getFrequency());
            telemetry.addLine("--- raw counts since A ---");
            telemetry.addData("X pod (forward)", dx);
            telemetry.addData("Y pod (strafe)", dy);
            telemetry.addData("X ticks per mm", "%.2f  (if rolled %.0f mm)", Math.abs(dx) / CAL_DISTANCE_MM, CAL_DISTANCE_MM);
            telemetry.addData("Y ticks per mm", "%.2f  (if rolled %.0f mm)", Math.abs(dy) / CAL_DISTANCE_MM, CAL_DISTANCE_MM);
            telemetry.addLine("--- fused pose (placeholder resolution) ---");
            telemetry.addData("X (mm)", "%.1f", pinpoint.getPosX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", "%.1f", pinpoint.getPosY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("A = zero counts, B = reset + recalibrate IMU (hold still)");
            telemetry.update();
        }
    }
}
