package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.FieldConstants.Alliance;
import org.firstinspires.ftc.teamcode.hardware.BallCounter;
import org.firstinspires.ftc.teamcode.hardware.BallPath;
import org.firstinspires.ftc.teamcode.hardware.CameraTilt;
import org.firstinspires.ftc.teamcode.hardware.Headlights;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.SavedNumber;
import org.firstinspires.ftc.teamcode.hardware.ShooterPresets;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.vision.HiveVision;

// One-driver TeleOp for the Nemesis turret robot (Oct 24 meet). Same
// buttons as the meet-1 TEMP TeleOp for driving, intake and shooting, plus
// the turret. Shot speed is the WALL number the auto uses, so tuning it here
// tunes the auto too.
//
//   Left stick           drive / strafe (up = toward the intake)
//   Right stick X        turn
//   L2 hold              slow mode
//   R2 hold              intake. Outtake OFF: shuts the gate, balls wait at
//                        it. Outtake ON: leaves the gate open, so R2 feeds
//                        balls straight through.
//   L1 hold              spit / unjam (reverse)
//   Cross                outtake on / off. Once it's up to speed (lightbar
//                        green) the gate opens by itself; off shuts it.
//   R1 hold              SHOOT: turns the outtake on if it's off, waits for
//                        it to get to speed, then opens the gate and feeds
//                        balls for as long as it's held.
//   Circle               gate open / shut
//   D-pad up / down      shot speed +/-2% (saved on the hub right away)
//   D-pad left / right   hold: turn the turret left / right.
//                        With auto-aim on: move the aim 1 deg left / right
//                        (saved, for when shots all miss to one side).
//   Square               turret back to FRONT (auto-aim off)
//   Triangle             auto-aim on / off: the turret follows the CELL's
//                        AprilTags with the Limelight; holds still when it
//                        can't see them
//   Options              aim at the other CELL (switch after every TIP)
//   Share                ball count back to 0
//   INIT: D-pad left = RED, right = BLUE (for auto-aim; remembered)
//
// Lightbar: red = outtake off, yellow = spinning up, green = ready.
// Rumble: 3 blips = holding 4 (FULL, stop intaking); long = more than 4
// (G407: spit one out NOW); 2 blips at 0:30 left; long at 0:15 (go park).
@TeleOp(name = "Nemesis TeleOp", group = "Nemesis")
public class NemesisTeleOp extends LinearOpMode {

    private static final double TELEOP_LENGTH_S = 120.0;
    private static final double SHOOT_NOW_WARNING_S = 30.0;
    private static final double PARK_NOW_WARNING_S = 15.0;

    private static final double PRECISION_SCALE = 0.35;
    private static final double TRIGGER_PRESSED = 0.5;
    private static final double SPEED_STEP = 0.02;

    // Manual turret speed while the D-pad is held.
    private static final double TURRET_MANUAL_DEG_PER_S = 90.0;

    // Auto-aim: the turret turns AIM_RATE x (tag offset) degrees per second
    // toward the tags. TODO: TUNE: lower if it wobbles, higher if it lags.
    private static final double AIM_RATE = 3.0;
    // "Locked" (telemetry) when the tags are this close to where we aim.
    private static final double AIM_TOLERANCE_DEG = 1.5;
    private static final double AIM_TRIM_STEP_DEG = 1.0;
    private static final String AIM_TRIM_FILE = "nemesis_aim_trim_deg";
    private static final String ALLIANCE_FILE = "nemesis_alliance";   // 0 = RED, 1 = BLUE

    private enum Light { OFF, RED, YELLOW, GREEN }

    private final MecanumDrive drive = new MecanumDrive();
    private final BallPath ballPath = new BallPath();
    private final Outtake outtake = new Outtake();
    private final Turret turret = new Turret();
    private final CameraTilt tilt = new CameraTilt();
    private final BallCounter counter = new BallCounter();
    private final Headlights headlights = new Headlights();
    private final HiveVision vision = new HiveVision();
    private final ShooterPresets presets = new ShooterPresets();

    private Light light = Light.OFF;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        ballPath.init(hardwareMap);
        outtake.init(hardwareMap);
        turret.init(hardwareMap);
        tilt.init(hardwareMap);
        counter.init(hardwareMap);
        headlights.init(hardwareMap);
        boolean hasLimelight = vision.initIfPresent(hardwareMap);
        presets.load();

        // Start position: turret facing front, Limelight at its saved tilt.
        turret.goFront();
        tilt.holdSavedPosition();

        Alliance alliance = SavedNumber.load(ALLIANCE_FILE, 0) > 0.5 ? Alliance.BLUE : Alliance.RED;
        while (opModeInInit()) {
            Alliance before = alliance;
            if (gamepad1.dpadLeftWasPressed()) alliance = Alliance.RED;
            if (gamepad1.dpadRightWasPressed()) alliance = Alliance.BLUE;
            if (alliance != before) SavedNumber.save(ALLIANCE_FILE, alliance == Alliance.BLUE ? 1 : 0);

            telemetry.addData("Alliance (D-pad left = RED, right = BLUE)", alliance);
            telemetry.addData("Shot speed", "%.0f%% (%s)", 100 * presets.get(ShooterPresets.Spot.WALL),
                    presets.isLoadedFromFile() ? "saved on hub" : "DEFAULT - not tuned");
            if (!ballPath.hasGate()) telemetry.addLine("WARNING: no 'shooter_gate' servo in the config");
            if (!ballPath.hasBallPathMotor()) telemetry.addLine("WARNING: no 'ball_path' motor in the config");
            if (!outtake.hasRoller()) telemetry.addLine("WARNING: no 'launcher_roller' motor in the config");
            if (!turret.hasBothServos()) telemetry.addLine("WARNING: turret servo(s) missing from the config");
            if (!counter.hasSensors()) telemetry.addLine("No ball sensors: no ball count");
            if (!hasLimelight) telemetry.addLine("No Limelight: no auto-aim");
            telemetry.addLine("Ready. R2 intake, R1 hold = shoot, Cross = outtake on/off.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        headlights.set(1.0);
        String saveNote = "saved on hub";
        double aimTrimDeg = SavedNumber.load(AIM_TRIM_FILE, 0);
        boolean outtakeOn = false;
        boolean gateHeldOpen = false;
        boolean wasReady = false;
        boolean autoAim = false;
        boolean aimSecondCell = false;
        double turretDeg = turret.getCurrentAngleDeg();
        int lastCount = counter.getCount();

        ElapsedTime matchTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        boolean warnedShoot = false, warnedPark = false;

        while (opModeIsActive()) {
            double dt = Math.min(loopTimer.seconds(), 0.1);
            loopTimer.reset();

            // ---- Drive ----
            double scale = gamepad1.left_trigger > TRIGGER_PRESSED ? PRECISION_SCALE : 1.0;
            drive.drive(-gamepad1.left_stick_y * scale, gamepad1.left_stick_x * scale, gamepad1.right_stick_x * scale);

            // ---- Shot speed (saved whenever it changes) ----
            double speedChange = 0;
            if (gamepad1.dpadUpWasPressed()) speedChange += SPEED_STEP;
            if (gamepad1.dpadDownWasPressed()) speedChange -= SPEED_STEP;
            if (speedChange != 0) {
                presets.trim(ShooterPresets.Spot.WALL, speedChange);
                saveNote = presets.save() ? "saved on hub" : "SAVE FAILED";
            }

            // ---- Outtake on/off; R1 turns it on too ----
            boolean shootHeld = gamepad1.right_bumper;
            if (gamepad1.crossWasPressed()) {
                outtakeOn = !outtakeOn;
                if (!outtakeOn) gateHeldOpen = false;
            }
            if (shootHeld) outtakeOn = true;
            if (outtakeOn) {
                outtake.setTarget(presets.get(ShooterPresets.Spot.WALL));
            } else {
                outtake.stop();
            }
            outtake.update();
            boolean ready = outtake.isReady();

            // ---- Ball path: spit beats shoot beats intake (as in TEMP) ----
            boolean intakeHeld = gamepad1.right_trigger > TRIGGER_PRESSED;
            boolean spitHeld = gamepad1.left_bumper;
            boolean feeding = shootHeld && ready && !spitHeld;
            if (ready && !wasReady) gateHeldOpen = true;   // outtake just got up to speed
            wasReady = ready;
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

            // ---- Ball count ----
            if (gamepad1.shareWasPressed()) counter.setCount(0);
            counter.update(ballPath.getIntakePower(), ballPath.isGateOpen());
            int count = counter.getCount();
            if (count > BallCounter.FULL && lastCount <= BallCounter.FULL) {
                gamepad1.rumble(800);
            } else if (count == BallCounter.FULL && lastCount < BallCounter.FULL) {
                gamepad1.rumbleBlips(3);
            }
            lastCount = count;

            // ---- Turret ----
            if (gamepad1.squareWasPressed()) {
                autoAim = false;
                turret.goFront();
                turretDeg = 0;
            }
            if (gamepad1.triangleWasPressed() && hasLimelight) autoAim = !autoAim;
            if (gamepad1.optionsWasPressed()) aimSecondCell = !aimSecondCell;

            vision.setTargetTags(aimSecondCell
                    ? FieldConstants.secondCellTags(alliance) : FieldConstants.firstCellTags(alliance));
            vision.update();

            // Read every loop so a press from manual mode isn't kept for later.
            boolean leftPressed = gamepad1.dpadLeftWasPressed();
            boolean rightPressed = gamepad1.dpadRightWasPressed();
            String aimState;
            if (autoAim) {
                double trimBefore = aimTrimDeg;
                // Aim further left = the tags sit further right in the frame.
                if (leftPressed) aimTrimDeg += AIM_TRIM_STEP_DEG;
                if (rightPressed) aimTrimDeg -= AIM_TRIM_STEP_DEG;
                if (aimTrimDeg != trimBefore) SavedNumber.save(AIM_TRIM_FILE, aimTrimDeg);

                if (vision.isTargetVisible()) {
                    // tx > 0: tags right of where we aim -> turn right (-).
                    double errorDeg = vision.getTargetTxDeg() - aimTrimDeg;
                    turretDeg -= AIM_RATE * errorDeg * dt;
                    aimState = Math.abs(errorDeg) < AIM_TOLERANCE_DEG ? "LOCKED" : "aiming";
                } else {
                    aimState = "can't see the tags (holding)";
                }
            } else {
                if (gamepad1.dpad_left) turretDeg += TURRET_MANUAL_DEG_PER_S * dt;
                if (gamepad1.dpad_right) turretDeg -= TURRET_MANUAL_DEG_PER_S * dt;
                aimState = "manual";
            }
            turret.setTargetAngleDeg(turretDeg);
            turretDeg = turret.getCurrentAngleDeg();   // clipped to the limits

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
            telemetry.addData("Outtake", "%s  speed %.0f%%  (%s)",
                    !outtakeOn ? "off" : ready ? "READY" : "spinning up",
                    100 * presets.get(ShooterPresets.Spot.WALL), saveNote);
            telemetry.addData("Flywheel", "measured %.0f%%%s%s", 100 * outtake.getMeasuredFraction(),
                    outtake.isEncoderFailed() ? "  ENCODER FAIL" : "",
                    outtake.isMaxedOut() ? "  MAXED OUT" : "");
            telemetry.addData("Turret", "%+.0f deg  %s%s", turretDeg,
                    autoAim ? "AUTO-AIM: " + aimState : aimState,
                    turret.isAtLeftLimit() ? "  (left limit)" : turret.isAtRightLimit() ? "  (right limit)" : "");
            telemetry.addData("Aiming at", "%s %s CELL  (aim trim %+.0f deg)", alliance,
                    aimSecondCell ? "SECOND" : "FIRST", aimTrimDeg);
            if (counter.hasSensors()) {
                telemetry.addData("Balls", "%d%s", count,
                        count > BallCounter.FULL ? "  TOO MANY - SPIT ONE (L1)"
                                : count == BallCounter.FULL ? "  FULL" : "");
            }
            telemetry.addData("Intake", intakeState);
            telemetry.addData("Gate", ballPath.isGateOpen() ? "OPEN" : "SHUT");
            telemetry.addData("Battery", "%.1f V", outtake.getVoltage());
            telemetry.update();
        }

        drive.stop();
        outtake.stop();
        outtake.update();
        ballPath.stop();
        headlights.set(0);
        vision.close();
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
