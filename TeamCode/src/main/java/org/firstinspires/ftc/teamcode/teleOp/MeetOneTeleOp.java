package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BallPath;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.ShooterPresets;

// ONE-DRIVER TeleOp for meet 1 (PS5 controller, gamepad 1). POLLEN only.
//
//   Left stick           drive / strafe (robot-centric: up = toward intake)
//   Right stick X        turn
//   L2 hold              precision mode (slow)
//   R2 hold              intake (gate stays shut, balls stack at the gate)
//   R1 press             SHOOT ALL: spins up, and once it's at speed fires
//                        all 4 back-to-back by itself (each ball waits for
//                        the flywheel to recover, so all 4 fly the same).
//                        Takes ~3-4 s -- hold still. Press R1 again to cancel.
//   L1 hold              spit / unjam (reverse; also cancels a volley) --
//                        use it the moment a 5th ball gets in (G407: never
//                        hold more than 4)
//   Cross                pre-spin on/off (drive to the wall already at speed)
//   D-pad up / down      shooting spot: WALL / ONE TILE
//   D-pad left / right   trim that spot's speed -1% / +1%
//   Share                save the speeds on the hub (auto uses them too)
//
// Lightbar: red = flywheel off, yellow = spinning up, green = ready.
// Rumble: 2 blips at 0:30 left (shoot what you hold), long at 0:15 (go
// park in the LOADING ZONE).
//
// Shoot from the WALL spot: back against the audience or far wall,
// centered on our HIVE, facing the upward CELL.
@TeleOp(name = "Meet 1 TeleOp", group = "teleOp")
public class MeetOneTeleOp extends LinearOpMode {

    private static final double TELEOP_LENGTH_S = 120.0;
    private static final double SHOOT_NOW_WARNING_S = 30.0;
    private static final double PARK_NOW_WARNING_S = 15.0;

    private static final double PRECISION_SCALE = 0.35;
    private static final double TRIGGER_PRESSED = 0.5;
    private static final double TRIM_STEP = 0.01;

    // Gate pulses per R1 press: 4 balls + 1 spare in case one didn't make it
    // through on its pulse (an empty pulse costs ~0.5 s).
    private static final int VOLLEY_PULSES = 5;

    private enum Light { OFF, RED, YELLOW, GREEN }

    private final MecanumDrive drive = new MecanumDrive();
    private final Flywheel flywheel = new Flywheel();
    private final BallPath ballPath = new BallPath();
    private final ShooterPresets presets = new ShooterPresets();

    private Light light = Light.OFF;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        flywheel.init(hardwareMap);
        ballPath.init(hardwareMap);
        presets.load();

        ShooterPresets.Spot spot = ShooterPresets.Spot.WALL;
        boolean preSpin = false;
        String saveNote = presets.isLoadedFromFile() ? "loaded from hub" : "defaults (not saved yet)";

        telemetry.addData("Shooter speeds", saveNote);
        telemetry.addData("WALL / ONE TILE", "%.0f%% / %.0f%%",
                100 * presets.get(ShooterPresets.Spot.WALL), 100 * presets.get(ShooterPresets.Spot.ONE_TILE));
        if (!ballPath.hasGate()) telemetry.addLine("WARNING: no 'shooter_gate' servo in the config!");
        telemetry.addLine("Ready. R2 intake, R1 shoot all (press once), L1 spit.");
        telemetry.update();

        waitForStart();
        ElapsedTime matchTimer = new ElapsedTime();
        boolean warnedShoot = false, warnedPark = false;

        while (opModeIsActive()) {
            // ---- Drive ----
            double scale = gamepad1.left_trigger > TRIGGER_PRESSED ? PRECISION_SCALE : 1.0;
            drive.drive(-gamepad1.left_stick_y * scale, gamepad1.left_stick_x * scale, gamepad1.right_stick_x * scale);

            // ---- Shooting spot, trim, save ----
            if (gamepad1.dpadUpWasPressed()) spot = ShooterPresets.Spot.WALL;
            if (gamepad1.dpadDownWasPressed()) spot = ShooterPresets.Spot.ONE_TILE;
            if (gamepad1.dpadLeftWasPressed()) presets.trim(spot, -TRIM_STEP);
            if (gamepad1.dpadRightWasPressed()) presets.trim(spot, TRIM_STEP);
            if (gamepad1.shareWasPressed()) {
                saveNote = presets.save() ? "SAVED on hub" : "SAVE FAILED";
                gamepad1.rumbleBlips(1);
            }
            if (gamepad1.crossWasPressed()) preSpin = !preSpin;

            // ---- Ball path: spit beats a volley beats intake ----
            boolean wasVolleying = ballPath.isVolleyActive();
            if (gamepad1.rightBumperWasPressed()) {
                if (wasVolleying) {
                    ballPath.setMode(BallPath.Mode.STOP);      // second press = cancel
                } else {
                    ballPath.startVolley(VOLLEY_PULSES);
                }
            }
            if (gamepad1.left_bumper) {
                ballPath.setMode(BallPath.Mode.SPIT);          // also cancels a volley
            } else if (!ballPath.isVolleyActive()) {
                ballPath.setMode(gamepad1.right_trigger > TRIGGER_PRESSED ? BallPath.Mode.INTAKE : BallPath.Mode.STOP);
            }

            // ---- Flywheel: runs during a volley, or all the time with pre-spin ----
            if (ballPath.isVolleyActive() || preSpin) {
                flywheel.setTarget(presets.get(spot));
            } else {
                flywheel.stop();
            }
            flywheel.update();
            ballPath.update(flywheel.isReady());
            if (wasVolleying && !ballPath.isVolleyActive()) gamepad1.rumble(200);   // volley over

            // ---- Driver feedback ----
            setLight(!flywheel.isSpinning() ? Light.RED : flywheel.isReady() ? Light.GREEN : Light.YELLOW);

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
            telemetry.addData("Spot", "%s  %.0f%%  (%s)", spot, 100 * presets.get(spot), saveNote);
            telemetry.addData("Flywheel", "%s  target %.0f%%  measured %.0f%%  (%.0f ticks/s)",
                    flywheel.isReady() ? "READY" : flywheel.isSpinning() ? "spinning up" : "off",
                    100 * flywheel.getTarget(), 100 * flywheel.getMeasuredFraction(),
                    flywheel.getMeasuredTicksPerSec());
            if (flywheel.isEncoderFailed()) telemetry.addLine("FLYWHEEL ENCODER FAIL - running open loop");
            if (flywheel.isMaxedOut()) telemetry.addLine("FLYWHEEL MAXED OUT - lower the speed or re-measure MAX_TICKS_PER_SEC");
            telemetry.addData("Pre-spin", preSpin ? "ON" : "off");
            if (ballPath.isVolleyActive()) {
                telemetry.addData("SHOOTING", "%d of %d fired%s", ballPath.getShotsFired(), ballPath.getVolleySize(),
                        flywheel.isReady() ? "" : " (waiting for speed)");
            } else {
                telemetry.addData("Ball path", "%s, gate %s", ballPath.getMode(), ballPath.isGateOpen() ? "OPEN" : "shut");
            }
            telemetry.addData("Battery", "%.1f V", flywheel.getVoltage());
            telemetry.update();
        }

        drive.stop();
        flywheel.stop();
        flywheel.update();
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
