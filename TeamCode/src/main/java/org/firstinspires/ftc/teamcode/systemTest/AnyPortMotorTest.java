package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Runs whatever motors are plugged into a hub's 4 motor ports. It talks to
// the hub's motor ports directly, so the motors DON'T need to be in the
// robot config (no names) -- 1, 2, 3 or all 4 plugged in all work, and an
// empty port just does nothing.
//   INIT: D-pad up = Control Hub, D-pad down = Expansion Hub
//   Left stick up/down   run every ON port together (up = forward)
//   Cross / Circle / Square / Triangle   port 0 / 1 / 2 / 3 on/off
//                        (all start ON; turn some off to test one at a time)
// Telemetry per port: power, current draw (a motor is plugged in if it draws
// current while the stick is pushed) and encoder count (changes if that
// motor's encoder cable is plugged in too).
@TeleOp(name = "Any Port Motor Test", group = "systemTest")
public class AnyPortMotorTest extends LinearOpMode {

    private static final int PORTS = 4;
    // Drawing more than this while powered = something is plugged in.
    private static final double PLUGGED_IN_AMPS = 0.1;
    private static final double STICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean useControlHub = true;

        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) useControlHub = true;
            if (gamepad1.dpadDownWasPressed()) useControlHub = false;
            LynxModule hub = findHub(useControlHub);
            telemetry.addData("Hub (D-pad up/down)", "%s%s", useControlHub ? "Control Hub" : "Expansion Hub",
                    hub == null ? "  -- NOT FOUND" : "");
            telemetry.addLine("Plug motors into any of its motor ports 0-3, press play,");
            telemetry.addLine("then push the left stick.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        LynxModule hub = findHub(useControlHub);
        if (hub == null) {
            telemetry.addLine("That hub isn't connected / in the config. Stop and pick the other one.");
            telemetry.update();
            while (opModeIsActive()) idle();
            return;
        }

        // Our own controller for this hub's motor ports: no config names needed.
        LynxDcMotorController controller;
        try {
            controller = new LynxDcMotorController(hardwareMap.appContext, hub);
        } catch (Exception e) {
            telemetry.addData("Couldn't open the hub's motor ports", e.toString());
            telemetry.update();
            while (opModeIsActive()) idle();
            return;
        }
        DcMotorEx[] motors = new DcMotorEx[PORTS];
        boolean[] on = new boolean[PORTS];
        for (int port = 0; port < PORTS; port++) {
            motors[port] = new DcMotorImplEx(controller, port);
            motors[port].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[port].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[port].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            on[port] = true;
        }

        try {
            while (opModeIsActive()) {
                if (gamepad1.crossWasPressed()) on[0] = !on[0];
                if (gamepad1.circleWasPressed()) on[1] = !on[1];
                if (gamepad1.squareWasPressed()) on[2] = !on[2];
                if (gamepad1.triangleWasPressed()) on[3] = !on[3];

                // Joystick y is negative when pushed up, so flip it.
                double stick = -gamepad1.left_stick_y;
                double power = Math.abs(stick) < STICK_DEADBAND ? 0 : stick;

                telemetry.addData("Hub", useControlHub ? "Control Hub" : "Expansion Hub");
                for (int port = 0; port < PORTS; port++) {
                    double p = on[port] ? power : 0;
                    motors[port].setPower(p);
                    double amps = motors[port].getCurrent(CurrentUnit.AMPS);
                    String plugged = p == 0 ? "" : amps > PLUGGED_IN_AMPS ? "  motor plugged in" : "  (nothing drawing current)";
                    telemetry.addData("Port " + port, "%s  power %+.2f  %.2f A  encoder %d%s",
                            on[port] ? "ON " : "off", p, amps, motors[port].getCurrentPosition(), plugged);
                }
                telemetry.addLine("Left stick = run. Cross/Circle/Square/Triangle = port 0/1/2/3 on/off.");
                telemetry.update();
            }
        } finally {
            for (DcMotorEx m : motors) {
                if (m != null) m.setPower(0);
            }
        }
    }

    // The Control Hub's own controller is the "parent" hub; an Expansion
    // Hub is not.
    private LynxModule findHub(boolean controlHub) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent() == controlHub) return module;
        }
        return null;
    }
}
