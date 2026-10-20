package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

// Tests whatever servo is plugged into Control Hub servo port 0. Like Single
// Wheel Test, it finds the servo by its port, not its config name.
//   Left joystick up/down  up = one way, down = the other way, let go = stop.
//                          Full stick = full speed.
// Works for both kinds of servo:
//   - continuous-rotation servo: the stick sets its speed (centered = stopped).
//   - normal servo: the stick sets its angle (centered = middle of its range).
@TeleOp(name = "Servo Spin Test", group = "systemTest")
public class ServoSpinTest extends LinearOpMode {

    private static final int PORT = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Configured as a Servo or as a CRServo -- either one works.
        Servo servo = findOnControlHub(Servo.class);
        CRServo crServo = servo == null ? findOnControlHub(CRServo.class) : null;
        HardwareDevice found = servo != null ? servo : crServo;
        if (found == null) {
            telemetry.addLine("No servo in the robot config on Control Hub servo port " + PORT + ".");
            telemetry.update();
            waitForStart();
            return;
        }
        String name = hardwareMap.getNamesOf(found).iterator().next();

        telemetry.addData("Servo", "%s (%s)", name, found.getConnectionInfo());
        telemetry.addLine("Ready. Press play, then move the LEFT joystick up/down.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Joystick y is negative when pushed up, so flip it.
            double stick = -gamepad1.left_stick_y;
            double position = 0.5 + 0.5 * stick;   // 0.0 .. 1.0, 0.5 = stop / middle
            if (servo != null) {
                servo.setPosition(position);
            } else {
                crServo.setPower(stick);
            }

            telemetry.addData("Servo", "%s (Control Hub servo port %d)", name, PORT);
            telemetry.addData("Stick", "%.2f", stick);
            telemetry.addData(servo != null ? "Position sent" : "Power sent", "%.2f",
                    servo != null ? position : stick);
            telemetry.update();
        }

        if (servo != null) {
            servo.setPosition(0.5);
        } else {
            crServo.setPower(0);
        }
    }

    // The configured device of this type on Control Hub servo port PORT, or
    // null. The Control Hub's own controller is the "parent" hub; an
    // Expansion Hub (also has a port 0) is not.
    private <T extends HardwareDevice> T findOnControlHub(Class<T> type) {
        Integer controlHubAddress = null;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent()) controlHubAddress = module.getModuleAddress();
        }
        if (controlHubAddress == null) return null;

        // Connection info looks like "USB (embedded); module 173; port 0".
        String wanted = "module " + controlHubAddress + "; port " + PORT;
        for (T device : hardwareMap.getAll(type)) {
            if (device.getConnectionInfo().endsWith(wanted)) return device;
        }
        return null;
    }
}
