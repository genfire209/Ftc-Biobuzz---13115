package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Tests whatever motor is plugged into Control Hub motor port 0. It finds
// the motor by its port, not its config name, so renaming motors in the
// robot config doesn't change which port it drives.
//   Left joystick up/down  motor speed: up = forward, down = reverse,
//                          let go = stop (full stick = full power).
@TeleOp(name = "Single Wheel Test", group = "systemTest")
public class SingleWheelTest extends LinearOpMode {

    private static final int PORT = 0;

    private DcMotor wheel;

    @Override
    public void runOpMode() throws InterruptedException {
        wheel = findControlHubMotor(PORT);
        if (wheel == null) {
            telemetry.addLine("No motor in the robot config on Control Hub motor port " + PORT + ".");
            telemetry.update();
            waitForStart();
            return;
        }
        String name = hardwareMap.getNamesOf(wheel).iterator().next();

        wheel.setDirection(DcMotor.Direction.FORWARD);
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Motor", "%s (%s)", name, wheel.getConnectionInfo());
        telemetry.addLine("Ready. Press play, then move the LEFT joystick up/down.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Joystick y is negative when pushed up, so flip it.
            double power = -gamepad1.left_stick_y;
            wheel.setPower(power);

            telemetry.addData("Motor", "%s (Control Hub port %d)", name, PORT);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Encoder", wheel.getCurrentPosition());
            telemetry.update();
        }

        wheel.setPower(0);
    }

    // The configured motor on this Control Hub motor port, or null. The
    // Control Hub's own motor controller is the "parent" hub; an Expansion
    // Hub (also has a port 0) is not.
    private DcMotor findControlHubMotor(int port) {
        Integer controlHubAddress = null;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent()) controlHubAddress = module.getModuleAddress();
        }
        if (controlHubAddress == null) return null;

        // Connection info looks like "USB (embedded); module 173; port 0".
        String wanted = "module " + controlHubAddress + "; port " + port;
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            if (motor.getConnectionInfo().endsWith(wanted)) return motor;
        }
        return null;
    }
}
