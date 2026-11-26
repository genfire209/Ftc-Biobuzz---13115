package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.Range;

// Turret on a goBILDA 312 RPM motor + shooter on a Matrix 12V motor, both on
// the CONTROL HUB. Like Any Port Motor Test, it talks to the hub's motor ports
// directly, so the motors DON'T need names in the robot config.
//   Control Hub motor port 0   turret  (312 RPM motor; encoder cable -> encoder port 0)
//   Control Hub motor port 1   shooter (Matrix 12V motor)
// PS5 controller (gamepad 1):
//   Left stick, left/right    spin the turret that way, full stick = full speed
//                             (let go = stop and hold)
//   Right stick, up/down      run the shooter: up = forward, down = backward
//                             (let go = shooter off). Full stick = the top
//                             speed set with the buttons below.
//   Triangle                  top speed +1% (up to 100%)
//   Cross (X)                 top speed -1% (down to 1%)
// Start the OpMode with the turret facing FRONT: that spot becomes encoder 0.
@TeleOp(name = "Turret + Shooter Test", group = "systemTest")
public class TurretShooterTest extends LinearOpMode {

    private static final int TURRET_PORT = 0;
    private static final int SHOOTER_PORT = 1;

    private static final double STICK_DEADBAND = 0.05;
    // Optional turret limit, in encoder ticks each way from the start position,
    // so the wires can't wind up. 0 = no limit. Find the number by turning the
    // turret to the furthest safe spot and reading "Turret ticks" on telemetry.
    private static final int TURRET_LIMIT_TICKS = 0;

    // Each Triangle / Cross press changes the shooter's top speed by this many percent.
    private static final int SHOOTER_STEP_PERCENT = 1;
    private static final int SHOOTER_MIN_PERCENT = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule controlHub = null;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent()) controlHub = module;
        }
        LynxDcMotorController controller = null;
        String problem = null;
        if (controlHub == null) {
            problem = "Control Hub not found in the robot config.";
        } else {
            try {
                controller = new LynxDcMotorController(hardwareMap.appContext, controlHub);
            } catch (Exception e) {
                problem = "Couldn't open the Control Hub's motor ports: " + e;
            }
        }
        if (controller == null) {
            telemetry.addLine(problem);
            telemetry.update();
            waitForStart();
            return;
        }

        DcMotorEx turret = new DcMotorImplEx(controller, TURRET_PORT);
        DcMotorEx shooter = new DcMotorImplEx(controller, SHOOTER_PORT);

        // If a stick turns the turret the wrong way, change FORWARD to REVERSE.
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // If the shooter spins backward, change FORWARD to REVERSE.
        shooter.setDirection(DcMotor.Direction.FORWARD);
        // FLOAT: the shooter coasts down when turned off instead of slamming to a stop.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int shooterMaxPercent = 100;

        telemetry.addLine("Ready. Control Hub port 0 = turret, port 1 = shooter.");
        telemetry.addLine("Turret should be facing FRONT.");
        telemetry.addLine("Left stick = turret, right stick = shooter");
        telemetry.addLine("Triangle = faster, X = slower (top speed for the stick)");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                // --- Turret: left stick left/right ---
                double turretPower = gamepad1.left_stick_x;
                if (Math.abs(turretPower) < STICK_DEADBAND) {
                    turretPower = 0;
                }

                int turretTicks = turret.getCurrentPosition();
                boolean atLimit = false;
                if (TURRET_LIMIT_TICKS > 0) {
                    if ((turretTicks >= TURRET_LIMIT_TICKS && turretPower > 0)
                            || (turretTicks <= -TURRET_LIMIT_TICKS && turretPower < 0)) {
                        turretPower = 0;
                        atLimit = true;
                    }
                }
                turret.setPower(turretPower);

                // --- Shooter: right stick runs it (y is negative when up);
                // Triangle / Cross set how fast full stick is. ---
                if (gamepad1.triangleWasPressed()) shooterMaxPercent += SHOOTER_STEP_PERCENT;
                if (gamepad1.crossWasPressed()) shooterMaxPercent -= SHOOTER_STEP_PERCENT;
                shooterMaxPercent = Range.clip(shooterMaxPercent, SHOOTER_MIN_PERCENT, 100);
                double shooterStick = -gamepad1.right_stick_y;
                if (Math.abs(shooterStick) < STICK_DEADBAND) {
                    shooterStick = 0;
                }
                double shooterPower = shooterStick * shooterMaxPercent / 100.0;
                shooter.setPower(shooterPower);

                telemetry.addData("Shooter top speed", "%d%%", shooterMaxPercent);
                telemetry.addData("Shooter power", "%+.0f%%", shooterPower * 100);
                telemetry.addData("Turret power", "%.2f", turretPower);
                telemetry.addData("Turret ticks", turretTicks);
                if (atLimit) {
                    telemetry.addLine("Turret at its limit -- push the other way");
                }
                telemetry.update();
            }
        } finally {
            turret.setPower(0);
            shooter.setPower(0);
        }
    }
}
