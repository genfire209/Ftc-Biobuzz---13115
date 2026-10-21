package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.BallCounter;
import org.firstinspires.ftc.teamcode.hardware.BallPath;
import org.firstinspires.ftc.teamcode.hardware.CameraTilt;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.SavedNumber;
import org.firstinspires.ftc.teamcode.hardware.ShooterPresets;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.vision.HiveVision;

// Sets up the Nemesis turret robot. Every number is saved on the hub the
// moment you set it, and every OpMode uses it from its next INIT (no
// rebuild). OPTIONS = next page. Do the pages in order the first time.
//
// 1 TURRET   D-pad left/right = turn (big step), L1/R1 = small step.
//            Cross    = save FRONT     (shooter pointing at the intake side)
//            Square   = save LEFT 90   (pointing exactly at the robot's LEFT)
//            Triangle = save LEFT LIMIT, Circle = save RIGHT LIMIT
//                       (as far as it may go each way: wires slack, nothing hit)
// 2 TRIM     Turret at FRONT. D-pad left/right = move the RIGHT servo a
//            hair; Cross = save. Set it where both servos go quiet (no
//            buzz) and the turret doesn't spring back when pushed lightly.
// 3 GATE     D-pad up/down = move the gate. Square = save SHUT, Triangle =
//            save OPEN. D-pad left/right = pulse length; Cross = one pulse
//            (right = exactly one ball gets through). R2/L2 = intake in/out.
// 4 OUTTAKE  Right stick up = flywheel power, left stick up = roller power
//            (raw). Telemetry: ticks/s and the highest seen -> put them in
//            Flywheel.MAX_TICKS_PER_SEC / Outtake.ROLLER_MAX_TICKS_PER_SEC
//            (full stick, charged battery, no ball). L1 = reset highest.
//            Cross = TEST SHOTS at the saved speed: R1 hold feeds balls,
//            R2 intakes, D-pad up/down = roller ratio (saved).
// 5 TILT     D-pad up/down = Limelight tilt (L1/R1 small); Cross = save.
//            From where you shoot, tilt until the HIVE tags are in view.
// 6 COUNTER  Live sensor distances. Hold a ball at each sensor and note the
//            number, then set "near" between ball and empty with D-pad
//            up/down (saved). R2/L2 = intake in/out, Share = count to 0.
@TeleOp(name = "Nemesis Setup Test", group = "systemTest")
public class NemesisSetupTest extends LinearOpMode {

    private enum Page { TURRET, TRIM, GATE, OUTTAKE, TILT, COUNTER }

    private static final double BIG_STEP = 0.01;
    private static final double SMALL_STEP = 0.002;
    private static final double TRIM_STEP = 0.002;
    private static final double GATE_STEP = 0.02;
    private static final double PULSE_STEP_S = 0.05;
    private static final double RATIO_STEP = 0.05;
    private static final double NEAR_STEP_MM = 2.0;
    private static final double TRIGGER_PRESSED = 0.5;

    private final Turret turret = new Turret();
    private final CameraTilt tilt = new CameraTilt();
    private final BallCounter counter = new BallCounter();
    private final Outtake outtake = new Outtake();
    private final HiveVision vision = new HiveVision();
    private final ShooterPresets presets = new ShooterPresets();

    private DcMotor intake;
    private DcMotor ballPathMotor;
    private Servo gate;
    private DcMotorEx flywheelMotor;
    private DcMotorEx rollerMotor;

    private String saveNote = "";

    @Override
    public void runOpMode() throws InterruptedException {
        turret.init(hardwareMap);
        tilt.init(hardwareMap);
        counter.init(hardwareMap);
        outtake.init(hardwareMap);
        boolean hasLimelight = vision.initIfPresent(hardwareMap);
        presets.load();

        // Raw access for the gate and the motors (BallPath would fight the
        // raw gate positions).
        intake = hardwareMap.get(DcMotor.class, BallPath.INTAKE_NAME);
        intake.setDirection(BallPath.INTAKE_DIRECTION);
        ballPathMotor = hardwareMap.tryGet(DcMotor.class, BallPath.BALL_PATH_NAME);
        if (ballPathMotor != null) ballPathMotor.setDirection(BallPath.BALL_PATH_DIRECTION);
        gate = hardwareMap.tryGet(Servo.class, "shooter_gate");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, Flywheel.MOTOR_NAME);
        rollerMotor = hardwareMap.tryGet(DcMotorEx.class, Outtake.ROLLER_NAME);

        double turretPosition = turret.getFrontPosition();
        double rightTrim = turret.getRightTrim();
        double shut = BallPath.savedGateClosed();
        double open = BallPath.savedGateOpen();
        double pulseS = BallPath.savedOpenPulseS();
        double gatePosition = shut;
        boolean pulsing = false;
        ElapsedTime pulseTimer = new ElapsedTime();
        double tiltPosition = CameraTilt.savedPosition();
        double rollerRatio = Outtake.savedRollerRatio();
        double nearMm = counter.getNearMm();
        boolean testShots = false;
        double flywheelMax = 0, rollerMax = 0;

        Page page = Page.TURRET;
        turret.setPosition(turretPosition);
        if (gate != null) gate.setPosition(gatePosition);
        tilt.setPosition(tiltPosition);

        telemetry.addLine("Nemesis Setup Test. OPTIONS = next page.");
        if (!turret.hasBothServos()) telemetry.addLine("WARNING: turret_left / turret_right missing from the config");
        if (gate == null) telemetry.addLine("WARNING: no 'shooter_gate' servo in the config");
        if (rollerMotor == null) telemetry.addLine("WARNING: no 'launcher_roller' motor in the config");
        if (ballPathMotor == null) telemetry.addLine("WARNING: no 'ball_path' motor in the config");
        if (!tilt.hasServo()) telemetry.addLine("WARNING: no 'camera_tilt' servo in the config");
        if (!counter.hasSensors()) telemetry.addLine("WARNING: no ball sensors in the config");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.optionsWasPressed()) {
                page = Page.values()[(page.ordinal() + 1) % Page.values().length];
                saveNote = "";
                if (testShots) {
                    testShots = false;
                    outtake.stop();
                    outtake.update();
                }
            }
            telemetry.addData("PAGE", "%d of %d: %s   (OPTIONS = next)", page.ordinal() + 1, Page.values().length, page);

            // Turret: raw on the TURRET page, held at FRONT everywhere else.
            if (page == Page.TURRET) {
                if (gamepad1.dpadRightWasPressed()) turretPosition += BIG_STEP;
                if (gamepad1.dpadLeftWasPressed()) turretPosition -= BIG_STEP;
                if (gamepad1.rightBumperWasPressed()) turretPosition += SMALL_STEP;
                if (gamepad1.leftBumperWasPressed()) turretPosition -= SMALL_STEP;
                turretPosition = Range.clip(turretPosition, 0, 1);
                turret.setPosition(turretPosition);
                turretPage(turretPosition);
            } else {
                turretPosition = turret.getFrontPosition();
                turret.setPosition(turretPosition);
            }

            double intakePower = 0;
            if (gamepad1.right_trigger > TRIGGER_PRESSED) intakePower = BallPath.INTAKE_POWER;
            if (gamepad1.left_trigger > TRIGGER_PRESSED) intakePower = BallPath.SPIT_POWER;
            double gateCommand = shut;

            switch (page) {
                case TRIM: {
                    double before = rightTrim;
                    if (gamepad1.dpadRightWasPressed()) rightTrim += TRIM_STEP;
                    if (gamepad1.dpadLeftWasPressed()) rightTrim -= TRIM_STEP;
                    if (rightTrim != before) turret.setRightTrim(rightTrim);
                    if (gamepad1.crossWasPressed()) {
                        saveNote = SavedNumber.save(Turret.RIGHT_TRIM_FILE, rightTrim) ? "TRIM saved" : "SAVE FAILED";
                    }
                    telemetry.addLine("Turret at FRONT. Move the RIGHT servo until both go quiet.");
                    telemetry.addData("Right servo trim", "%+.3f   (Cross = save)", rightTrim);
                    break;
                }
                case GATE: {
                    if (gamepad1.dpadUpWasPressed()) gatePosition = Range.clip(gatePosition + GATE_STEP, 0, 1);
                    if (gamepad1.dpadDownWasPressed()) gatePosition = Range.clip(gatePosition - GATE_STEP, 0, 1);
                    if (gamepad1.squareWasPressed()) {
                        shut = gatePosition;
                        saveNote = SavedNumber.save(BallPath.GATE_CLOSED_FILE, shut) ? "SHUT saved" : "SAVE FAILED";
                    }
                    if (gamepad1.triangleWasPressed()) {
                        open = gatePosition;
                        saveNote = SavedNumber.save(BallPath.GATE_OPEN_FILE, open) ? "OPEN saved" : "SAVE FAILED";
                    }
                    boolean shorter = gamepad1.dpadLeftWasPressed();
                    boolean longer = gamepad1.dpadRightWasPressed();
                    if (shorter) pulseS = Math.max(PULSE_STEP_S, pulseS - PULSE_STEP_S);
                    if (longer) pulseS += PULSE_STEP_S;
                    if (shorter || longer) {
                        saveNote = SavedNumber.save(BallPath.OPEN_PULSE_FILE, pulseS) ? "pulse saved" : "SAVE FAILED";
                    }
                    if (gamepad1.crossWasPressed() && !pulsing) {
                        pulsing = true;
                        pulseTimer.reset();
                    }
                    gateCommand = gatePosition;
                    if (pulsing) {
                        gateCommand = pulseTimer.seconds() < pulseS ? open : shut;
                        if (pulseTimer.seconds() >= pulseS) {
                            pulsing = false;
                            gatePosition = shut;
                        }
                    }
                    telemetry.addData("Gate position now", "%.2f", gateCommand);
                    telemetry.addData("SHUT / OPEN (saved)", "%.2f / %.2f", shut, open);
                    telemetry.addData("Pulse length (saved)", "%.2f s", pulseS);
                    break;
                }
                case OUTTAKE: {
                    if (gamepad1.crossWasPressed()) {
                        testShots = !testShots;
                        if (!testShots) {
                            outtake.stop();
                            outtake.update();
                        }
                    }
                    if (gamepad1.leftBumperWasPressed()) {
                        flywheelMax = 0;
                        rollerMax = 0;
                    }
                    if (testShots) {
                        double before = rollerRatio;
                        if (gamepad1.dpadUpWasPressed()) rollerRatio += RATIO_STEP;
                        if (gamepad1.dpadDownWasPressed()) rollerRatio -= RATIO_STEP;
                        rollerRatio = Range.clip(rollerRatio, Outtake.MIN_ROLLER_RATIO, Outtake.MAX_ROLLER_RATIO);
                        if (rollerRatio != before) {
                            outtake.setRollerRatio(rollerRatio);
                            saveNote = SavedNumber.save(Outtake.ROLLER_RATIO_FILE, rollerRatio) ? "ratio saved" : "SAVE FAILED";
                        }
                        outtake.setTarget(presets.get(ShooterPresets.Spot.WALL));
                        outtake.update();
                        boolean feed = gamepad1.right_bumper && outtake.isReady();
                        if (feed) {
                            intakePower = BallPath.FEED_POWER;
                            gateCommand = open;
                        }
                        telemetry.addData("TEST SHOTS", "%s at the saved speed %.0f%%  (R1 hold = feed, Cross = stop)",
                                outtake.isReady() ? "READY" : "spinning up", 100 * presets.get(ShooterPresets.Spot.WALL));
                        telemetry.addData("Roller ratio", "%.2f  (D-pad up/down)", rollerRatio);
                    } else {
                        flywheelMotor.setPower(Range.clip(-gamepad1.right_stick_y, 0, 1));
                        if (rollerMotor != null) rollerMotor.setPower(Range.clip(-gamepad1.left_stick_y, 0, 1));
                        telemetry.addLine("Raw power: right stick = flywheel, left stick = roller. Cross = test shots.");
                    }
                    double fly = Math.abs(flywheelMotor.getVelocity());
                    double rol = rollerMotor != null ? Math.abs(rollerMotor.getVelocity()) : 0;
                    flywheelMax = Math.max(flywheelMax, fly);
                    rollerMax = Math.max(rollerMax, rol);
                    telemetry.addData("Flywheel ticks/s", "%.0f  (highest %.0f -> Flywheel.MAX_TICKS_PER_SEC)", fly, flywheelMax);
                    telemetry.addData("Roller ticks/s", rollerMotor == null ? "no roller motor"
                            : String.format("%.0f  (highest %.0f -> Outtake.ROLLER_MAX_TICKS_PER_SEC)", rol, rollerMax));
                    telemetry.addLine("ticks/s 0 at full power = that motor's encoder isn't plugged in.");
                    break;
                }
                case TILT: {
                    if (gamepad1.dpadUpWasPressed()) tiltPosition += BIG_STEP;
                    if (gamepad1.dpadDownWasPressed()) tiltPosition -= BIG_STEP;
                    if (gamepad1.rightBumperWasPressed()) tiltPosition += SMALL_STEP;
                    if (gamepad1.leftBumperWasPressed()) tiltPosition -= SMALL_STEP;
                    tiltPosition = Range.clip(tiltPosition, 0, 1);
                    tilt.setPosition(tiltPosition);
                    if (gamepad1.crossWasPressed()) {
                        saveNote = SavedNumber.save(CameraTilt.SAVED_POSITION_FILE, tiltPosition) ? "TILT saved" : "SAVE FAILED";
                    }
                    vision.update();
                    telemetry.addData("Tilt position", "%.3f   (Cross = save)", tiltPosition);
                    telemetry.addData("Limelight", hasLimelight
                            ? vision.countAllVisible() + " AprilTag(s) in view" : "not in the config");
                    break;
                }
                case COUNTER: {
                    double before = nearMm;
                    if (gamepad1.dpadUpWasPressed()) nearMm += NEAR_STEP_MM;
                    if (gamepad1.dpadDownWasPressed()) nearMm -= NEAR_STEP_MM;
                    nearMm = Math.max(NEAR_STEP_MM, nearMm);
                    if (nearMm != before) {
                        counter.setNearMm(nearMm);
                        saveNote = SavedNumber.save(BallCounter.NEAR_FILE, nearMm) ? "near saved" : "SAVE FAILED";
                    }
                    if (gamepad1.shareWasPressed()) counter.setCount(0);
                    counter.update(intakePower, false);
                    telemetry.addData("Ramp sensor (A)", counter.hasRampSensor()
                            ? String.format("%.0f mm", counter.getRampMm()) : "missing");
                    telemetry.addData("Gate sensor (B)", counter.hasGateSensor()
                            ? String.format("%.0f mm%s", counter.getGateMm(), counter.isBallAtGate() ? "  BALL" : "")
                            : "missing");
                    telemetry.addData("Near = under", "%.0f mm  (D-pad up/down)", nearMm);
                    telemetry.addData("Ball count", "%d  (Share = 0)", counter.getCount());
                    break;
                }
                default:
                    break;
            }

            if (page != Page.OUTTAKE) {
                flywheelMotor.setPower(0);
                if (rollerMotor != null) rollerMotor.setPower(0);
            }
            if (page != Page.TILT) tilt.setPosition(tiltPosition);
            if (gate != null) gate.setPosition(gateCommand);
            intake.setPower(intakePower);
            if (ballPathMotor != null) ballPathMotor.setPower(intakePower);

            telemetry.addData("Last save", saveNote);
            telemetry.update();
        }

        flywheelMotor.setPower(0);
        if (rollerMotor != null) rollerMotor.setPower(0);
        intake.setPower(0);
        if (ballPathMotor != null) ballPathMotor.setPower(0);
        vision.close();
    }

    private void turretPage(double position) {
        if (gamepad1.crossWasPressed()) {
            saveNote = SavedNumber.save(Turret.FRONT_FILE, position) ? "FRONT saved" : "SAVE FAILED";
            turret.loadCalibration();
        }
        if (gamepad1.squareWasPressed()) {
            saveNote = SavedNumber.save(Turret.LEFT_90_FILE, position) ? "LEFT 90 saved" : "SAVE FAILED";
            turret.loadCalibration();
        }
        double angle = turret.angleOf(position);
        if (gamepad1.triangleWasPressed()) {
            saveNote = angle <= 0 ? "LEFT LIMIT must be LEFT of front (angle > 0)"
                    : SavedNumber.save(Turret.LEFT_LIMIT_FILE, angle) ? "LEFT LIMIT saved" : "SAVE FAILED";
            turret.loadCalibration();
        }
        if (gamepad1.circleWasPressed()) {
            saveNote = angle >= 0 ? "RIGHT LIMIT must be RIGHT of front (angle < 0)"
                    : SavedNumber.save(Turret.RIGHT_LIMIT_FILE, angle) ? "RIGHT LIMIT saved" : "SAVE FAILED";
            turret.loadCalibration();
        }
        telemetry.addLine("Order: Cross FRONT, Square LEFT 90, then Triangle/Circle limits.");
        telemetry.addData("Servo position", "%.3f", position);
        telemetry.addData("Turret angle", "%+.0f deg (+ = left)", turret.angleOf(position));
        telemetry.addData("Saved FRONT", "%.3f", turret.getFrontPosition());
        telemetry.addData("Saved limits", "left %+.0f / right %+.0f deg",
                turret.getLeftLimitDeg(), turret.getRightLimitDeg());
    }
}
