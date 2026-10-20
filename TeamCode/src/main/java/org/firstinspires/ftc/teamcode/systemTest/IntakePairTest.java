package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Runs the two intake motors on motor ports 0 and 1 together. Like Single
// Wheel Test, it finds the motors by port, not by config name.
//   Before START (INIT): D-pad up = Control Hub, D-pad down = Expansion Hub
//   Right trigger   both motors forward (press harder = faster)
//   Left trigger    both motors backward
//   Square          flip the port 1 motor's direction (if the two fight each other)
@TeleOp(name = "Intake Pair Test", group = "systemTest")
public class IntakePairTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        boolean useControlHub = true;
        DcMotor port0 = null;
        DcMotor port1 = null;

        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) useControlHub = true;
            if (gamepad1.dpadDownWasPressed()) useControlHub = false;
            port0 = findMotor(useControlHub, 0);
            port1 = findMotor(useControlHub, 1);

            telemetry.addData("Hub (D-pad up/down)", useControlHub ? "Control Hub" : "Expansion Hub");
            telemetry.addData("Port 0", describe(port0));
            telemetry.addData("Port 1", describe(port1));
            telemetry.addLine("Press play, then hold the right trigger.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        for (DcMotor m : new DcMotor[] {port0, port1}) {
            if (m == null) continue;
            m.setDirection(DcMotor.Direction.FORWARD);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        boolean flipPort1 = false;

        while (opModeIsActive()) {
            if (gamepad1.squareWasPressed()) flipPort1 = !flipPort1;

            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            if (port0 != null) port0.setPower(power);
            if (port1 != null) port1.setPower(flipPort1 ? -power : power);

            telemetry.addData("Hub", useControlHub ? "Control Hub" : "Expansion Hub");
            telemetry.addData("Port 0", describe(port0));
            telemetry.addData("Port 1", "%s%s", describe(port1), flipPort1 ? "  (FLIPPED)" : "");
            telemetry.addData("Power", "%.2f", power);
            telemetry.addLine("R2 = forward, L2 = backward, Square = flip port 1");
            telemetry.update();
        }

        if (port0 != null) port0.setPower(0);
        if (port1 != null) port1.setPower(0);
    }

    private String describe(DcMotor m) {
        return m == null ? "NOTHING in the robot config on this port" : hardwareMap.getNamesOf(m).iterator().next();
    }

    // The configured motor on this port of the Control Hub (the "parent" hub)
    // or the Expansion Hub, or null.
    private DcMotor findMotor(boolean controlHub, int port) {
        Integer address = null;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent() == controlHub) address = module.getModuleAddress();
        }
        if (address == null) return null;

        // Connection info looks like "USB (embedded); module 173; port 0".
        String wanted = "module " + address + "; port " + port;
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            if (motor.getConnectionInfo().endsWith(wanted)) return motor;
        }
        return null;
    }
}
