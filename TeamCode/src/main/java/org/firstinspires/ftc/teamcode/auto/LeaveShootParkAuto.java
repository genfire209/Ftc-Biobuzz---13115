package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.BallPath;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.SavedNumber;
import org.firstinspires.ftc.teamcode.hardware.ShooterPresets;

// LEAVE + SHOOT + PARK with no odometry pods and no Limelight. Distances come
// from the drive motors' own encoders; the robot never turns (only drives
// and strafes), and it keeps the wheels from twisting so it stays square.
//
// START (works for both alliances - the field is rotationally symmetric):
//   RED:  back against the AUDIENCE wall, centered on our HIVE, shooter
//         facing the HIVE.  BLUE: same, against the FAR wall.
// THEN:
//   1. forward LEAVE_IN toward the HIVE (LEAVE), shooter spinning up
//   2. shoot the preloads: the intake feeds them into the shooter in short
//      bursts so the shooter gets back to speed between balls (no gate)
//   3. strafe left into the lane at x = 33 (clear of the HIVE, the floor bar
//      and the FLOWER), drive up past the HIVE, strafe left into the
//      LOADING ZONE (partly inside = PARK)
// Shot speed = the WALL speed you tune in Nemesis TeleOp (D-pad up/down).
//
// INIT menu (gamepad 1):
//   D-pad up/down  start delay          Circle    park on/off
//   D-pad left/right  shoot OFF / ON (remembered) - OFF = just LEAVE + PARK
//   Triangle       park in the near / far half of the LOADING ZONE
//   CALIBRATE (do once): push the robot by hand exactly 1 tile (24 in)
//     FORWARD then press Cross; push it exactly 1 tile to the RIGHT then press
//     Square. Saved on the hub. Telemetry shows the live distance.
// Drive: Control Hub ports 1 FL, 0 FR, 3 BL, 2 BR (as in Drive Only TeleOp).
@Autonomous(name = "Leave+Shoot+Park (no pods)", group = "Nemesis", preselectTeleOp = "Nemesis TeleOp")
public class LeaveShootParkAuto extends LinearOpMode {

    private static final int FL_PORT = 1, FR_PORT = 0, BL_PORT = 3, BR_PORT = 2;

    // Field (red; blue is the same moves rotated). FieldConstants: HIVE line
    // x 59.25, floor bar x 47.3-49.3, left FLOWER x 0-7.6, LOADING ZONE
    // x <= 12.9, y 95.5-119.0.
    private static final double HIVE_X = 59.25;
    private static final double LANE_X = 33.3;
    private static final double LZ_X_MAX = 12.9;
    private static final double LZ_NEAR_Y = 101.0, LZ_FAR_Y = 113.0;
    private static final double WALL_IN = 1.3;
    // Robot, center (between the wheels) to its edges. Measured 2026-10-24.
    private static final double BACK_IN = 6.25;
    private static final double LEFT_IN = 5.5;
    private static final double PARK_OVERLAP_IN = 5.0;   // how far inside the zone

    private static final double LEAVE_IN = 12.0;

    // Encoder ticks per inch. Default: goBILDA 312 RPM (537.7 ticks/turn) on
    // a 104 mm wheel, 1:1 belt. Calibrate in INIT (saved on the hub).
    private static final String SHOOT_FILE = "auto_shoot";              // 1 = shoot, 0 = don't
    private static final String TPI_FWD_FILE = "auto_ticks_per_in_fwd";
    private static final String TPI_STRAFE_FILE = "auto_ticks_per_in_strafe";
    private static final double DEFAULT_TPI = 537.7 / (Math.PI * 104.0 / 25.4);
    private static final double CAL_DISTANCE_IN = 24.0;

    // Motion
    private static final double MAX_POWER = 0.5;
    private static final double MIN_POWER = 0.12;
    private static final double KP = 0.05;            // power per inch of error
    private static final double KP_CROSS = 0.05;      // keep the other axis at 0
    private static final double KP_TWIST = 0.002;     // power per tick of twist
    private static final double RAMP_PER_S = 1.5;     // power ramp-up limit
    private static final double TOLERANCE_IN = 0.5;

    // Shooting (no gate): spin up, then feed in bursts.
    private static final double SPINUP_MAX_S = 3.0;
    private static final int FEED_BURSTS = 5;          // 4 balls + 1 spare
    private static final double FEED_ON_S = 0.45;
    private static final double FEED_OFF_S = 0.7;

    private static final double STOP_BY_S = 29.0;
    private static final double MAX_DELAY_S = 10.0;

    private DcMotor fl, fr, bl, br;
    private int fl0, fr0, bl0, br0;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private String step = "INIT";

    @Override
    public void runOpMode() throws InterruptedException {
        // ---- drive by port ----
        LynxModule controlHub = null;
        for (LynxModule m : hardwareMap.getAll(LynxModule.class)) {
            if (m.isParent()) controlHub = m;
        }
        if (controlHub == null) {
            telemetry.addLine("Control Hub not found.");
            telemetry.update();
            waitForStart();
            return;
        }
        LynxDcMotorController ports;
        try {
            ports = new LynxDcMotorController(hardwareMap.appContext, controlHub);
        } catch (RobotCoreException e) {
            telemetry.addData("Couldn't open the drive ports", e.toString());
            telemetry.update();
            waitForStart();
            return;
        }
        fl = new DcMotorImplEx(ports, FL_PORT, MecanumDrive.FRONT_LEFT_DIRECTION);
        fr = new DcMotorImplEx(ports, FR_PORT, MecanumDrive.FRONT_RIGHT_DIRECTION);
        bl = new DcMotorImplEx(ports, BL_PORT, MecanumDrive.BACK_LEFT_DIRECTION);
        br = new DcMotorImplEx(ports, BR_PORT, MecanumDrive.BACK_RIGHT_DIRECTION);
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // easy to push in INIT
        }

        // ---- shooter + feed (by config name; optional) ----
        Outtake outtake = null;
        BallPath feed = null;
        String shooterNote = "";
        try {
            outtake = new Outtake();
            outtake.init(hardwareMap);
        } catch (Exception e) {
            outtake = null;
            shooterNote += "no 'launcher_pollen' in the config -> won't shoot. ";
        }
        try {
            feed = new BallPath();
            feed.init(hardwareMap);
        } catch (Exception e) {
            feed = null;
            shooterNote += "no 'intake' in the config -> won't feed. ";
        }
        ShooterPresets presets = new ShooterPresets();
        presets.load();

        double tpiFwd = SavedNumber.load(TPI_FWD_FILE, DEFAULT_TPI);
        double tpiStrafe = SavedNumber.load(TPI_STRAFE_FILE, DEFAULT_TPI);
        double delayS = 0;
        boolean park = true;
        boolean shoot = SavedNumber.load(SHOOT_FILE, 1) > 0.5;
        boolean farHalf = true;
        String calNote = "";
        markStart();

        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) delayS = Math.min(MAX_DELAY_S, delayS + 1);
            if (gamepad1.dpadDownWasPressed()) delayS = Math.max(0, delayS - 1);
            if (gamepad1.circleWasPressed()) park = !park;
            boolean shootBefore = shoot;
            if (gamepad1.dpadLeftWasPressed()) shoot = false;
            if (gamepad1.dpadRightWasPressed()) shoot = true;
            if (shoot != shootBefore) SavedNumber.save(SHOOT_FILE, shoot ? 1 : 0);
            if (gamepad1.triangleWasPressed()) farHalf = !farHalf;
            double fwdTicks = fwdTicks(), strafeTicks = strafeTicks();
            if (gamepad1.crossWasPressed()) {
                if (Math.abs(fwdTicks) > 100) {
                    tpiFwd = Math.abs(fwdTicks) / CAL_DISTANCE_IN;
                    calNote = SavedNumber.save(TPI_FWD_FILE, tpiFwd) ? "FORWARD saved" : "SAVE FAILED";
                } else {
                    calNote = "push it forward 1 tile first";
                }
                markStart();
            }
            if (gamepad1.squareWasPressed()) {
                if (Math.abs(strafeTicks) > 100) {
                    tpiStrafe = Math.abs(strafeTicks) / CAL_DISTANCE_IN;
                    calNote = SavedNumber.save(TPI_STRAFE_FILE, tpiStrafe) ? "SIDEWAYS saved" : "SAVE FAILED";
                } else {
                    calNote = "push it right 1 tile first";
                }
                markStart();
            }

            telemetry.addLine("RED: back on the AUDIENCE wall | BLUE: back on the FAR wall");
            telemetry.addLine("Centered on our HIVE, shooter facing it.");
            telemetry.addData("Start delay (D-pad up/down)", "%.0f s", delayS);
            telemetry.addData("Shoot (D-pad left/right)", shoot ? "ON - shoots the preloads" : "OFF - LEAVE + PARK only");
            telemetry.addData("Park (Circle)", park ? "YES - LOADING ZONE" : "no");
            telemetry.addData("Park half (Triangle)", farHalf ? "FAR half of the zone" : "NEAR half of the zone");
            telemetry.addData("Shot speed", "%.0f%% (Nemesis TeleOp D-pad sets it)", 100 * presets.get(ShooterPresets.Spot.WALL));
            telemetry.addLine("-- calibrate: push 1 tile, then Cross (forward) / Square (right) --");
            telemetry.addData("Pushed", "forward %+.1f in, right %+.1f in", fwdTicks / tpiFwd, strafeTicks / tpiStrafe);
            telemetry.addData("Ticks per inch", "forward %.1f, sideways %.1f  %s", tpiFwd, tpiStrafe, calNote);
            if (!shooterNote.isEmpty()) telemetry.addLine("WARNING: " + shooterNote);
            telemetry.update();
        }
        if (isStopRequested()) return;
        matchTimer.reset();
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            step = "start delay";
            sleepActive(delayS);

            // 1. LEAVE, with the shooter spinning up on the way
            if (shoot && outtake != null) outtake.setTarget(presets.get(ShooterPresets.Spot.WALL));
            step = "forward (LEAVE)";
            move(LEAVE_IN, 0, tpiFwd, tpiStrafe, outtake);

            // 2. shoot the preloads
            if (shoot && outtake != null && feed != null) {
                step = "shooter getting to speed";
                ElapsedTime t = new ElapsedTime();
                while (opModeIsActive() && t.seconds() < SPINUP_MAX_S && !outtake.isReady()) {
                    outtake.update();
                    show();
                }
                for (int b = 0; b < FEED_BURSTS && opModeIsActive() && matchTimer.seconds() < 20; b++) {
                    step = "shooting " + (b + 1) + " of " + FEED_BURSTS;
                    runFeed(feed, outtake, BallPath.FEED_POWER, FEED_ON_S);
                    runFeed(feed, outtake, 0, FEED_OFF_S);
                }
            }
            if (outtake != null) {
                outtake.stop();
                outtake.update();
            }
            if (feed != null) feed.stop();

            // 3. PARK in the LOADING ZONE (red numbers; blue is the same moves)
            if (park) {
                double startY = WALL_IN + BACK_IN + LEAVE_IN;
                double parkX = LZ_X_MAX - PARK_OVERLAP_IN + LEFT_IN;
                step = "strafe left into the lane";
                move(0, -(HIVE_X - LANE_X), tpiFwd, tpiStrafe, null);
                step = "drive up past the HIVE";
                move((farHalf ? LZ_FAR_Y : LZ_NEAR_Y) - startY, 0, tpiFwd, tpiStrafe, null);
                step = "strafe left into the LOADING ZONE";
                move(0, -(LANE_X - parkX), tpiFwd, tpiStrafe, null);
            }
            step = "done";
            stopDrive();
            while (opModeIsActive()) show();
        } finally {
            stopDrive();
            if (outtake != null) {
                outtake.stop();
                outtake.update();
            }
            if (feed != null) feed.stop();
        }
    }

    // Drive (forward, right) inches in the robot frame, one axis at a time,
    // holding the other axis and the twist at zero. Stops by STOP_BY_S.
    private void move(double forwardIn, double rightIn, double tpiFwd, double tpiStrafe, Outtake spinning) {
        markStart();
        boolean forwardMove = Math.abs(forwardIn) >= Math.abs(rightIn);
        double target = forwardMove ? forwardIn : rightIn;
        double timeout = 1.5 + Math.abs(target) / 12.0;   // generous: ~12 in/s at worst
        ElapsedTime t = new ElapsedTime();
        double ramp = 0;
        while (opModeIsActive() && t.seconds() < timeout && matchTimer.seconds() < STOP_BY_S) {
            double fwd = fwdTicks() / tpiFwd, right = strafeTicks() / tpiStrafe;
            double along = forwardMove ? fwd : right;
            double across = forwardMove ? right : fwd;
            double err = target - along;
            if (Math.abs(err) < TOLERANCE_IN) break;
            ramp = Math.min(MAX_POWER, ramp + RAMP_PER_S * 0.02);
            double p = Range.clip(KP * err, -ramp, ramp);
            if (Math.abs(p) < MIN_POWER) p = Math.copySign(MIN_POWER, err);
            double fix = -KP_CROSS * across;
            double twist = -KP_TWIST * twistTicks();
            if (forwardMove) setDrive(p, fix, twist);
            else setDrive(fix, p, twist);
            if (spinning != null) spinning.update();
            show();
        }
        stopDrive();
    }

    private void runFeed(BallPath feed, Outtake outtake, double power, double seconds) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            feed.setManual(power, true);
            feed.update(true);
            outtake.update();
            show();
        }
    }

    private void setDrive(double forward, double right, double turnCw) {
        double a = forward + right + turnCw, b = forward - right - turnCw;
        double c = forward - right + turnCw, d = forward + right - turnCw;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d))));
        fl.setPower(a / max);
        fr.setPower(b / max);
        bl.setPower(c / max);
        br.setPower(d / max);
    }

    private void stopDrive() {
        if (fl == null) return;
        setDrive(0, 0, 0);
    }

    private void markStart() {
        fl0 = fl.getCurrentPosition();
        fr0 = fr.getCurrentPosition();
        bl0 = bl.getCurrentPosition();
        br0 = br.getCurrentPosition();
    }

    // Wheel ticks since markStart(), split into forward / right / twist.
    private double fwdTicks() {
        return ((fl.getCurrentPosition() - fl0) + (fr.getCurrentPosition() - fr0)
                + (bl.getCurrentPosition() - bl0) + (br.getCurrentPosition() - br0)) / 4.0;
    }

    private double strafeTicks() {
        return ((fl.getCurrentPosition() - fl0) - (fr.getCurrentPosition() - fr0)
                - (bl.getCurrentPosition() - bl0) + (br.getCurrentPosition() - br0)) / 4.0;
    }

    private double twistTicks() {
        return ((fl.getCurrentPosition() - fl0) - (fr.getCurrentPosition() - fr0)
                + (bl.getCurrentPosition() - bl0) - (br.getCurrentPosition() - br0)) / 4.0;
    }

    private void sleepActive(double seconds) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) show();
    }

    private void show() {
        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f s", matchTimer.seconds());
        telemetry.update();
    }
}
