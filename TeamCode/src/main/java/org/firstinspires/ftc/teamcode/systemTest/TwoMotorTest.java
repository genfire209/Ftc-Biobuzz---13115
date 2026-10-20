package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Runs the two motors on ports 0 and 1 together (e.g. two intake motors).
// Like Single Wheel Test, it finds the motors by port, not by config name.
//   INIT: D-pad left/right   pick the hub: Control Hub or Expansion Hub
//   Left joystick up/down    both motors: up = forward, down = reverse, let go = stop
//   Cross                    reverse the port 1 motor (for motors that face
//                            each other, so both spin the intake the same way)
@TeleOp(name = "Two Motor Test (ports 0 + 1)", group = "systemTest")
public class TwoMotorTest extends LinearOpMode {

    private static final int PORT_A = 0;
    private static final int PORT_B = 1;

    private Integer controlHubAddress;
    private Integer expansionHubAddress;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent()) {
                controlHubAddress = module.getModuleAddress();
            } else {
                expansionHubAddress = module.getModuleAddress();
            }
        }

        boolean useExpansionHub = false;
        DcMotor motorA = null, motorB = null;
        while (opModeInInit()) {
            boolean left = gamepad1.dpadLeftWasPressed();
            boolean right = gamepad1.dpadRightWasPressed();
            if (left || right) useExpansionHub = !useExpansionHub;

            Integer hub = useExpansionHub ? expansionHubAddress : controlHubAddress;
            motorA = findMotor(hub, PORT_A);
            motorB = findMotor(hub, PORT_B);

            String hubName = useExpansionHub ? "EXPANSION HUB" : "CONTROL HUB";
            telemetry.addData("Hub (D-pad left/right)", hub == null ? hubName + " - not connected" : hubName);
            telemetry.addData("Port " + PORT_A, describe(motorA));
            telemetry.addData("Port " + PORT_B, describe(motorB));
            telemetry.addLine("Press play, then move the LEFT joystick up/down.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        for (DcMotor motor : new DcMotor[] {motorA, motorB}) {
            if (motor == null) continue;
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        boolean reverseB = false;
        while (opModeIsActive()) {
            if (gamepad1.crossWasPressed()) reverseB = !reverseB;

            // Joystick y is negative when pushed up, so flip it.
            double power = -gamepad1.left_stick_y;
            if (motorA != null) motorA.setPower(power);
            if (motorB != null) motorB.setPower(reverseB ? -power : power);

            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Port " + PORT_A, describe(motorA));
            telemetry.addData("Port " + PORT_B, "%s%s", describe(motorB), reverseB ? "  (REVERSED - Cross)" : "");
            telemetry.update();
        }

        if (motorA != null) motorA.setPower(0);
        if (motorB != null) motorB.setPower(0);
    }

    private String describe(DcMotor motor) {
        return motor == null ? "nothing in the robot config on this port" : hardwareMap.getNamesOf(motor).iterator().next();
    }

    // The configured motor on this hub and port, or null.
    private DcMotor findMotor(Integer hubAddress, int port) {
        if (hubAddress == null) return null;
        // Connection info looks like "USB (embedded); module 173; port 0".
        String wanted = "module " + hubAddress + "; port " + port;
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            if (motor.getConnectionInfo().endsWith(wanted)) return motor;
        }
        return null;
    }
}
