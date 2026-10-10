package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.FieldConstants.Alliance;
import org.firstinspires.ftc.teamcode.hardware.BallPath;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.hardware.ShooterPresets;

import java.util.ArrayList;
import java.util.List;

// MEET 1 AUTO. Kept deliberately simple: no Pedro, no vision, no turret.
// Driving uses the Pinpoint pose and a plain go-to-point controller, which
// needs no tuning beyond the pod directions.
//
// SHOOT + PARK (red; blue is the same rotated 180 deg):
//   Start with the robot's BACK touching the audience wall in column C,
//   centered on the red HIVE (x = 59.25), facing it, 4 POLLEN behind the
//   shut gate. Fires all 4 (+1 spare gate pulse) at the upward CELL -- it
//   already holds 3 NECTAR, so 3 POLLEN should TIP it (20) and the 4th is
//   insurance against a miss. Then drives straight off the wall (LEAVE, 3),
//   over to a lane at x = 30, up the lane and into the LOADING ZONE
//   (PARK, 5).
//
// PARK ONLY (for when the partner shoots the preloads):
//   Start with the robot's LEFT side touching the red alliance wall, front
//   just short of the LOADING ZONE, facing the far wall. Slides off the wall
//   (LEAVE) and forward into the zone (PARK). Keeps its POLLEN for TELEOP.
//
// INIT menu (gamepad 1): D-pad up/down = start delay, Circle = park on/off,
// Triangle = which half of the LOADING ZONE (agree with the partner).
// The INIT screen also shows how far the robot has been pushed -- use it to
// check the odometry pods (push forward 1 tile -> "forward +23.5").
//
// No Pinpoint (or it reports a fault): still shoots, then drives off the
// wall on a timer for LEAVE and stops. It won't try to park blind.
public abstract class MeetOneAutoBase extends OpMode {

    protected Alliance alliance;
    protected boolean shootPreload;

    // ---- Robot size from the CAD, measured from the robot center (middle
    // of the four wheel axles). TODO: MEASURE on the real robot -- the
    // Matrix motor at the back may stick out further than the CAD's motor.
    private static final double FRONT_IN = 8.1;   // center -> intake edge
    private static final double BACK_IN = 7.2;    // center -> shooter end
    private static final double LEFT_IN = 6.4;    // center -> left side (battery)

    private static final double FACING_FAR_WALL = Math.toRadians(90);

    // ---- Red poses (FieldConstants frame, inches) ----
    private static final Pose RED_SHOOT_START = new Pose(
            FieldConstants.RED_HIVE_X, FieldConstants.WALL_IN + BACK_IN, FACING_FAR_WALL);
    // TODO: TUNE at the practice HIVE. 0 = shoot right from the wall (uses
    // the WALL speed). If the robot can't make the shot from the wall, set
    // this to 23.5 and AUTO_SHOT_SPOT to ONE_TILE.
    private static final double SHOOT_FORWARD_IN = 0.0;
    private static final ShooterPresets.Spot AUTO_SHOT_SPOT = ShooterPresets.Spot.WALL;

    // Off the wall far enough to count as LEAVE and to turn the corner.
    private static final double LEAVE_Y = FieldConstants.WALL_IN + BACK_IN + 10.0;
    // Lane up the red side: clears the A-frame bar (x >= 47.3) and the left
    // FLOWER (x <= 7.6) by more than 10in either side of the robot.
    private static final double LANE_X = 30.0;
    // LOADING ZONE (red tile A5): x up to 12.9, y 95.5-119. The robot ends
    // with its left side 3in inside the tape ("at least partially in").
    private static final double LZ_X_MAX = 12.9;
    private static final double PARK_X = LZ_X_MAX - 3.0 + LEFT_IN;
    private static final double PARK_Y_AUDIENCE_HALF = 96.0;
    private static final double PARK_Y_FAR_HALF = 118.0;

    // Park only: left side on the alliance wall, front 1in short of the zone
    // (starting inside it is illegal, G304E).
    private static final Pose RED_PARK_ONLY_START = new Pose(
            FieldConstants.WALL_IN + LEFT_IN, FieldConstants.RED_LZ_Y_MIN - FRONT_IN - 1.0, FACING_FAR_WALL);
    private static final double PARK_ONLY_LEAVE_X = FieldConstants.WALL_IN + LEFT_IN + 4.0;

    // ---- Timing ----
    private static final double MAX_DELAY_S = 10.0;
    private static final double MAX_SHOOTING_S = 9.0;
    // Stop shooting and head for the LOADING ZONE no later than this.
    private static final double GO_PARK_BY_S = 24.0;
    // AUTO is 30s; everything is stopped before the buzzer (G403).
    private static final double STOP_BY_S = 29.5;
    private static final int PRELOAD_BALLS = 4;
    private static final int SPARE_PULSES = 1;

    // ---- Go-to-point controller ----
    private static final double KP_DRIVE = 0.05;           // power per inch of error
    private static final double KP_TURN = 1.0;             // power per radian of error
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MIN_DRIVE_POWER = 0.12;    // enough to keep it creeping in
    private static final double MAX_TURN_POWER = 0.3;
    private static final double PASS_TOLERANCE_IN = 3.0;   // waypoints along the way
    private static final double FINAL_TOLERANCE_IN = 1.5;  // last waypoint
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(4);
    // A leg that takes longer than base + distance/speed is skipped, so one
    // stuck leg can't eat the rest of AUTO.
    private static final double LEG_TIMEOUT_BASE_S = 1.0;
    private static final double LEG_TIMEOUT_IN_PER_S = 12.0;

    // No odometry: drive off the wall for this long (LEAVE only).
    private static final double BLIND_LEAVE_POWER = 0.4;
    private static final double BLIND_LEAVE_S = 0.6;

    // Robot must sit still this long after INIT for the gyro to calibrate.
    private static final double GYRO_SETTLE_S = 1.0;

    private enum Step { DELAY, TO_SHOOT_SPOT, FIRING, ROUTE, BLIND_LEAVE, DONE }

    private final MecanumDrive drive = new MecanumDrive();
    private final Flywheel flywheel = new Flywheel();
    private final BallPath ballPath = new BallPath();
    private final Odometry odometry = new Odometry();
    private final ShooterPresets presets = new ShooterPresets();

    // INIT menu choices.
    private double delayS = 0;
    private boolean park = true;
    private boolean parkFarHalf = false;

    private Step step;
    private boolean useOdometry;
    private Pose start;
    private final List<Pose> route = new ArrayList<>();
    private int routeIndex;
    private double legTimeoutS;
    private String note = "";

    private final ElapsedTime initTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void init() {
        drive.init(hardwareMap);
        flywheel.init(hardwareMap);
        ballPath.init(hardwareMap);
        odometry.init(hardwareMap);
        presets.load();
        start = at(shootPreload ? RED_SHOOT_START : RED_PARK_ONLY_START);
        initTimer.reset();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) delayS = Math.min(MAX_DELAY_S, delayS + 1);
        if (gamepad1.dpadDownWasPressed()) delayS = Math.max(0, delayS - 1);
        if (gamepad1.circleWasPressed()) park = !park;
        if (gamepad1.triangleWasPressed()) parkFarHalf = !parkFarHalf;

        odometry.update();

        telemetry.addData("Auto", "%s %s", alliance, shootPreload ? "SHOOT + PARK" : "PARK ONLY");
        telemetry.addLine(shootPreload
                ? "Back on the " + (alliance == Alliance.RED ? "AUDIENCE" : "FAR") + " wall, centered on our HIVE, facing it."
                : "Left side on our alliance wall, just short of the LOADING ZONE.");
        telemetry.addData("Start delay (D-pad up/down)", "%.0f s", delayS);
        telemetry.addData("Park (Circle)", park ? "YES" : "no");
        telemetry.addData("Park half (Triangle)", parkFarHalf ? "FAR-wall half" : "AUDIENCE half");
        if (shootPreload) {
            telemetry.addData("Shot speed", "%.0f%% (%s)", 100 * presets.get(AUTO_SHOT_SPOT),
                    presets.isLoadedFromFile() ? "saved on hub" : "DEFAULT - not tuned");
            if (!ballPath.hasGate()) telemetry.addLine("WARNING: no 'shooter_gate' servo in the config!");
        }
        telemetry.addData("Pinpoint", odometry.getStatus());
        if (initTimer.seconds() < GYRO_SETTLE_S) {
            telemetry.addLine("Keep the robot still...");
        } else if (odometry.isPresent()) {
            // Until START the Pinpoint's frame is "where INIT was pressed":
            // x = forward, y = left. Push test for the pod directions.
            telemetry.addData("Push test", "forward %+.1f in, left %+.1f in, turned %+.0f deg",
                    odometry.getX(), odometry.getY(), Math.toDegrees(odometry.getHeading()));
        }
        telemetry.update();
    }

    @Override
    public void start() {
        matchTimer.reset();
        odometry.update();
        useOdometry = odometry.isReady();
        if (useOdometry) {
            odometry.setPose(start.getX(), start.getY(), start.getHeading());
        } else {
            note = "no odometry -> LEAVE only";
        }
        setStep(Step.DELAY);
    }

    @Override
    public void loop() {
        odometry.update();
        flywheel.update();
        ballPath.update(flywheel.isReady());
        double t = matchTimer.seconds();

        if (t > STOP_BY_S && step != Step.DONE) {
            note = "out of time";
            finish();
        }

        switch (step) {
            case DELAY:
                if (t < delayS) break;
                if (!shootPreload) {
                    beginRoute();
                } else {
                    flywheel.setTarget(presets.get(AUTO_SHOT_SPOT));
                    if (SHOOT_FORWARD_IN > 0 && useOdometry) {
                        legTimeoutS = LEG_TIMEOUT_BASE_S + SHOOT_FORWARD_IN / LEG_TIMEOUT_IN_PER_S;
                        setStep(Step.TO_SHOOT_SPOT);
                    } else {
                        beginFiring();
                    }
                }
                break;

            case TO_SHOOT_SPOT:
                if (driveTo(shootSpot(), FINAL_TOLERANCE_IN) || stepTimer.seconds() > legTimeoutS) {
                    drive.stop();
                    beginFiring();
                }
                break;

            case FIRING:
                boolean allFired = ballPath.getShotsFired() >= PRELOAD_BALLS + SPARE_PULSES && !ballPath.isGateOpen();
                if (allFired || stepTimer.seconds() > MAX_SHOOTING_S || t > GO_PARK_BY_S) {
                    if (!allFired) note = "stopped shooting early (time)";
                    flywheel.stop();
                    ballPath.stop();
                    beginRoute();
                }
                break;

            case ROUTE:
                followRoute();
                break;

            case BLIND_LEAVE:
                if (stepTimer.seconds() < BLIND_LEAVE_S) {
                    // Shoot start: back on the wall -> drive forward.
                    // Park only: left side on the wall -> strafe right.
                    if (shootPreload) {
                        drive.drive(BLIND_LEAVE_POWER, 0, 0);
                    } else {
                        drive.drive(0, BLIND_LEAVE_POWER, 0);
                    }
                } else {
                    finish();
                }
                break;

            case DONE:
                drive.stop();
                break;
        }

        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f s", t);
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.0f deg",
                odometry.getX(), odometry.getY(), Math.toDegrees(odometry.getHeading()));
        if (step == Step.ROUTE) telemetry.addData("Waypoint", "%d of %d", routeIndex + 1, route.size());
        telemetry.addData("Shots (gate pulses)", ballPath.getShotsFired());
        telemetry.addData("Flywheel", "target %.0f%%  measured %.0f%%%s",
                100 * flywheel.getTarget(), 100 * flywheel.getMeasuredFraction(),
                flywheel.isEncoderFailed() ? "  ENCODER FAIL" : "");
        telemetry.addData("Note", note);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        flywheel.stop();
        flywheel.update();
        ballPath.stop();
    }

    // ---- Steps ----

    private void beginFiring() {
        ballPath.resetShots();
        ballPath.setMode(BallPath.Mode.SHOOT);
        setStep(Step.FIRING);
    }

    private void beginRoute() {
        if (!useOdometry) {
            setStep(Step.BLIND_LEAVE);
            return;
        }
        route.clear();
        double parkY = parkFarHalf ? PARK_Y_FAR_HALF : PARK_Y_AUDIENCE_HALF;
        if (shootPreload) {
            double leaveY = Math.max(LEAVE_Y, RED_SHOOT_START.getY() + SHOOT_FORWARD_IN);
            route.add(at(new Pose(FieldConstants.RED_HIVE_X, leaveY, FACING_FAR_WALL)));
            if (park) {
                route.add(at(new Pose(LANE_X, leaveY, FACING_FAR_WALL)));
                route.add(at(new Pose(LANE_X, parkY, FACING_FAR_WALL)));
                route.add(at(new Pose(PARK_X, parkY, FACING_FAR_WALL)));
            }
        } else {
            route.add(at(new Pose(PARK_ONLY_LEAVE_X, RED_PARK_ONLY_START.getY(), FACING_FAR_WALL)));
            if (park) route.add(at(new Pose(PARK_X, parkY, FACING_FAR_WALL)));
        }
        routeIndex = 0;
        startLeg();
        setStep(Step.ROUTE);
    }

    private void followRoute() {
        boolean last = routeIndex == route.size() - 1;
        boolean arrived = driveTo(route.get(routeIndex), last ? FINAL_TOLERANCE_IN : PASS_TOLERANCE_IN);
        boolean timedOut = stepTimer.seconds() > legTimeoutS;
        if (!arrived && !timedOut) return;

        if (timedOut) note = "leg " + (routeIndex + 1) + " timed out, moving on";
        if (last) {
            finish();
        } else {
            routeIndex++;
            startLeg();
        }
    }

    private void startLeg() {
        Pose target = route.get(routeIndex);
        double distance = Math.hypot(target.getX() - odometry.getX(), target.getY() - odometry.getY());
        legTimeoutS = LEG_TIMEOUT_BASE_S + distance / LEG_TIMEOUT_IN_PER_S;
        stepTimer.reset();
    }

    private void finish() {
        drive.stop();
        flywheel.stop();
        ballPath.stop();
        setStep(Step.DONE);
    }

    // ---- Driving ----

    // One step toward target (field frame). Returns true once within
    // toleranceIn and HEADING_TOLERANCE_RAD (without stopping the motors, so
    // the next leg continues smoothly).
    private boolean driveTo(Pose target, double toleranceIn) {
        double heading = odometry.getHeading();
        double ex = target.getX() - odometry.getX();
        double ey = target.getY() - odometry.getY();
        double distance = Math.hypot(ex, ey);
        double headingError = AngleUnit.normalizeRadians(target.getHeading() - heading);
        if (distance < toleranceIn && Math.abs(headingError) < HEADING_TOLERANCE_RAD) return true;

        // Field error -> robot frame (forward along the heading, left 90 deg CCW of it).
        double forward = ex * Math.cos(heading) + ey * Math.sin(heading);
        double left = -ex * Math.sin(heading) + ey * Math.cos(heading);

        double f = KP_DRIVE * forward;
        double l = KP_DRIVE * left;
        double magnitude = Math.hypot(f, l);
        if (magnitude > MAX_DRIVE_POWER) {
            f *= MAX_DRIVE_POWER / magnitude;
            l *= MAX_DRIVE_POWER / magnitude;
        } else if (distance >= toleranceIn && magnitude > 1e-6 && magnitude < MIN_DRIVE_POWER) {
            f *= MIN_DRIVE_POWER / magnitude;
            l *= MIN_DRIVE_POWER / magnitude;
        }
        double turnCounterclockwise = Range.clip(KP_TURN * headingError, -MAX_TURN_POWER, MAX_TURN_POWER);

        drive.drive(f, -l, -turnCounterclockwise);
        return false;
    }

    private Pose shootSpot() {
        return at(new Pose(RED_SHOOT_START.getX(), RED_SHOOT_START.getY() + SHOOT_FORWARD_IN, FACING_FAR_WALL));
    }

    private void setStep(Step s) {
        step = s;
        stepTimer.reset();
    }

    private Pose at(Pose redPose) {
        return FieldConstants.forAlliance(redPose, alliance);
    }
}
