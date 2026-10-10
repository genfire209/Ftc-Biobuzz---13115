package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.BallPath;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SavedNumber;

// TEMPORARY one-driver TeleOp for when nothing is tuned (no odometry, no
// Limelight, no flywheel speed numbers). The outtake just runs at a power
// you pick -- saved on the hub -- and the driver aims by eye.
//
//   Left stick           drive / strafe (up = toward the intake)
//   Right stick X        turn
//   L2 hold              slow mode
//   R2 hold              intake. Outtake OFF: shuts the gate, balls wait at
//                        it. Outtake ON: leaves the gate alone -- open it
//                        with Circle and R2 feeds balls straight through.
//   L1 hold              spit / unjam (reverse) -- use it if a 5th ball
//                        gets in (G407: never hold more than 4). Same gate
//                        rule as R2.
//   Cross                outtake on / off
//   R1 hold              SHOOT: turns the outtake on if it's off, waits
//                        SPINUP_S for it to get to speed, then opens the
//                        gate and feeds balls for as long as it's held
//                        (let go: the gate goes back to how Circle left it).
//                        Tap it for one ball at a time -- more consistent,
//                        because each ball slows the flywheel down.
//   D-pad up / down      outtake power +/-5%   (saved on the hub right away)
//   D-pad left / right   outtake power -/+1%
//   Circle               gate open / shut. While the outtake is on, this is
//                        the only thing that shuts it.
//   Triangle / Square    move the gate +/- one step. Moves whichever
//                        position it's in now -- SHUT or OPEN -- and saves
//                        it on the hub (same numbers as Shooter Setup Test).
//
// Lightbar: red = outtake off, yellow = spinning up, green = ready.
// Rumble: 2 blips at 0:30 left, long at 0:15 (go park in the LOADING ZONE).
@TeleOp(name = "TEMP TeleOp (simple)", group = "Meet 1")
public class TempTeleOp extends LinearOpMode {

    private static final double TELEOP_LENGTH_S = 120.0;
    private static final double SHOOT_NOW_WARNING_S = 30.0;
    private static final double PARK_NOW_WARNING_S = 15.0;

    private static final double PRECISION_SCALE = 0.35;
    private static final double TRIGGER_PRESSED = 0.5;

    private static final String POWER_FILE = "meet1_temp_outtake_power";
    private static final double DEFAULT_POWER = 0.70;
    private static final double MIN_POWER = 0.20;
    private static final double BIG_STEP = 0.05;
    private static final double SMALL_STEP = 0.01;

    // The outtake power is scaled by NOMINAL_VOLTAGE / battery voltage, so
    // a setting throws about the same on a fresh or a tired battery.
    private static final double NOMINAL_VOLTAGE = 12.5;
    private static final double VOLTAGE_READ_PERIOD_S = 0.25;
    // Time from outtake on to the first ball (no speed sensor used here).
    private static final double SPINUP_S = 1.5;
    // Gate servo step per Triangle/Square press (~6 deg on a 300 deg servo).
    private static final double GATE_STEP = 0.02;

    private enum Light { OFF, RED, YELLOW, GREEN }

    private final MecanumDrive drive = new MecanumDrive();
    private final BallPath ballPath = new BallPath();

    private Light light = Light.OFF;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        ballPath.init(hardwareMap);

        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, Flywheel.MOTOR_NAME);
        outtake.setDirection(Flywheel.DIRECTION);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
        double voltage = NOMINAL_VOLTAGE;
        ElapsedTime voltageTimer = new ElapsedTime();

        double power = Range.clip(SavedNumber.load(POWER_FILE, DEFAULT_POWER), MIN_POWER, 1.0);
        String saveNote = "saved on hub";
        boolean outtakeOn = false;
        boolean gateHeldOpen = false;
        String gateNote = "";
        ElapsedTime spinTimer = new ElapsedTime();

        telemetry.addData("Outtake power", "%.0f%%", 100 * power);
        if (!ballPath.hasGate()) telemetry.addLine("WARNING: no 'shooter_gate' servo in the config!");
        telemetry.addLine("Ready. R2 intake, R1 hold = shoot, Cross = outtake on/off.");
        telemetry.update();

        waitForStart();
        ElapsedTime matchTimer = new ElapsedTime();
        boolean warnedShoot = false, warnedPark = false;

        while (opModeIsActive()) {
            // ---- Drive ----
            double scale = gamepad1.left_trigger > TRIGGER_PRESSED ? PRECISION_SCALE : 1.0;
            drive.drive(-gamepad1.left_stick_y * scale, gamepad1.left_stick_x * scale, gamepad1.right_stick_x * scale);

            // ---- Outtake power (saved whenever it changes) ----
            double before = power;
            if (gamepad1.dpadUpWasPressed()) power += BIG_STEP;
            if (gamepad1.dpadDownWasPressed()) power -= BIG_STEP;
            if (gamepad1.dpadRightWasPressed()) power += SMALL_STEP;
            if (gamepad1.dpadLeftWasPressed()) power -= SMALL_STEP;
            power = Range.clip(power, MIN_POWER, 1.0);
            if (power != before) {
                saveNote = SavedNumber.save(POWER_FILE, power) ? "saved on hub" : "SAVE FAILED";
            }

            // ---- Outtake on/off; R1 turns it on too ----
            boolean shootHeld = gamepad1.right_bumper;
            if (gamepad1.crossWasPressed()) {
                outtakeOn = !outtakeOn;
                spinTimer.reset();
            }
            if (shootHeld && !outtakeOn) {
                outtakeOn = true;
                spinTimer.reset();
            }
            boolean ready = outtakeOn && spinTimer.seconds() > SPINUP_S;

            if (battery != null && voltageTimer.seconds() > VOLTAGE_READ_PERIOD_S) {
                voltageTimer.reset();
                double v = battery.getVoltage();
                if (v > 6.0) voltage = v;     // ignore a bad (0 V) read
            }
            outtake.setPower(outtakeOn ? Range.clip(power * NOMINAL_VOLTAGE / voltage, 0, 1) : 0);

            // ---- Ball path: spit beats shoot beats intake ----
            // Gate: Circle opens/shuts it. Outtake off -> intake or spit
            // shuts it too; outtake on -> only Circle does. R1 opens it
            // while held.
            boolean intakeHeld = gamepad1.right_trigger > TRIGGER_PRESSED;
            boolean spitHeld = gamepad1.left_bumper;
            boolean feeding = shootHeld && ready && !spitHeld;
            if (gamepad1.circleWasPressed()) gateHeldOpen = !gateHeldOpen;
            if (!outtakeOn && (intakeHeld || spitHeld)) gateHeldOpen = false;

            double intakePower = 0;
            String intakeState = "stopped";
            if (spitHeld) {
                intakePower = BallPath.SPIT_POWER;
                intakeState = "OUT (spit)";
            } else if (feeding) {
                intakePower = BallPath.FEED_POWER;
                intakeState = "FEEDING the outtake";
            } else if (intakeHeld) {
                intakePower = BallPath.INTAKE_POWER;
                intakeState = "IN";
            }
            ballPath.setManual(intakePower, gateHeldOpen || feeding);
            ballPath.update(ready);

            // ---- Gate set-up: after update() so it moves the position the
            // gate is actually in now ----
            if (gamepad1.triangleWasPressed()) {
                gateNote = ballPath.nudgeGate(GATE_STEP) ? "saved" : "SAVE FAILED";
            }
            if (gamepad1.squareWasPressed()) {
                gateNote = ballPath.nudgeGate(-GATE_STEP) ? "saved" : "SAVE FAILED";
            }

            // ---- Driver feedback ----
            setLight(!outtakeOn ? Light.RED : ready ? Light.GREEN : Light.YELLOW);

            double timeLeft = TELEOP_LENGTH_S - matchTimer.seconds();
            if (!warnedShoot && timeLeft < SHOOT_NOW_WARNING_S) {
                gamepad1.rumbleBlips(2);
                warnedShoot = true;
            }
            if (!warnedPark && timeLeft < PARK_NOW_WARNING_S) {
                gamepad1.rumble(1000);
                warnedPark = true;
            }

            telemetry.addData("Time left", "%.0f s%s", Math.max(0, timeLeft),
                    timeLeft < PARK_NOW_WARNING_S ? "  -> PARK NOW" : "");
            telemetry.addData("Outtake", "%s  power %.0f%%  (%s)",
                    !outtakeOn ? "off" : ready ? "READY" : "spinning up", 100 * power, saveNote);
            telemetry.addData("Outtake speed", "%.0f ticks/s", Math.abs(outtake.getVelocity()));
            telemetry.addData("Intake", intakeState);
            telemetry.addData("Gate", "%s at %.2f  (Triangle/Square to move) %s",
                    ballPath.isGateOpen() ? "OPEN" : "SHUT", ballPath.getGatePosition(), gateNote);
            telemetry.addData("Battery", "%.1f V", voltage);
            telemetry.update();
        }

        drive.stop();
        outtake.setPower(0);
        ballPath.stop();
    }

    // Only sends a new color when it changes (each call queues a message to
    // the Driver Hub).
    private void setLight(Light wanted) {
        if (wanted == light) return;
        light = wanted;
        switch (wanted) {
            case RED:
                gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case YELLOW:
                gamepad1.setLedColor(1, 0.6, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case GREEN:
                gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            default:
                break;
        }
    }
}
