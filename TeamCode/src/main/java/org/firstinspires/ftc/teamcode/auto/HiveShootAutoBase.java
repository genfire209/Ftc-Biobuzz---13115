package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.FieldConstants.Alliance;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.HiveAimer;
import org.firstinspires.ftc.teamcode.vision.HiveVision;

import java.util.ArrayList;
import java.util.List;

// TWO-TIP AUTO for the shooting robot. The alliance partner runs a
// park-only auto (ParkOnlyAutoBase) and holds the audience half of the
// LOADING ZONE; this robot takes the other half at the end.
//
// Tip threshold (tested on a real HIVE, 2026-10-02): about 200g. The
// starting upward CELL already holds 3 NECTAR, so TIP 1 needs only 3
// POLLEN; every CELL after that starts empty and needs 8 POLLEN.
//
// Sequence (red; blue is the same rotated 180 deg):
//   1. At the start spot, fire 3 POLLEN at the first CELL -> TIP 1. Keep
//      the 4th: firing it while the HIVE is tipping wastes it and can stop
//      the TIP from counting. If vision shows TIP 1 did NOT happen (a
//      miss), fire the 4th at the same CELL.
//   2. Sweep the GARDEN for 3 POLLEN (4 if the spare was used), so the
//      robot never holds more than 4 (G407).
//   3. Drive to the far side of the HIVE and fire everything at the
//      second CELL (4/8).
//   4. Pull 4 POLLEN from the bottom of the left-wall FLOWER (G418B).
//   5. Drive back and fire 4 -> 8/8 -> TIP 2.
//   6. Park in the far-wall half of the LOADING ZONE.
//
// Built to go as fast as the robot can; time it on a real field and tune
// from there. Every drive runs at full power except the GARDEN sweep, the
// flywheel pre-spins while driving, and the turret aims while driving.
//
// TIMING SAFETY: if PARK_IS_REQUIRED, a leg only starts if there's still
// time to park afterward, and the robot abandons whatever it's doing to
// park when the clock gets that low. Set it false to always go for the
// second TIP (20 pts) even if that means missing AUTO PARK (5 pts).
public abstract class HiveShootAutoBase extends OpMode {

    protected Alliance alliance;

    // ---- Tuning (TODO: TUNE on a real field) ----
    private static final double AUTO_LENGTH_S = 30.0;
    private static final boolean PARK_IS_REQUIRED = true;

    // Balls needed to TIP (from the team's own HIVE test).
    private static final int BALLS_TO_TIP_FIRST_CELL = 3;   // CELL starts with 3 NECTAR
    private static final int BALLS_TO_TIP_EMPTY_CELL = 8;
    private static final int PRELOAD_BALLS = 4;
    private static final int MAX_HELD_BALLS = 4;            // G407

    private static final double SWEEP_MAX_POWER = 0.45;     // slow enough to swallow balls
    private static final double INTAKE_TAIL_S = 0.6;        // keep pulling after the sweep
    private static final double FLOWER_DWELL_S = 2.5;       // time to pull 4 out of the FLOWER

    // Aim: fire on vision lock; without tags, fire on the odometry angle
    // after AIM_FALLBACK_S instead of waiting forever.
    private static final double AIM_FALLBACK_S = 0.75;
    // Fire anyway after this long, so a turret hunting just outside the
    // tolerance can't stall the whole auto.
    private static final double AIM_GIVE_UP_S = 2.0;

    // TIP 1 confirmation (see waitForTip1()).
    private static final double TIP_CONFIRM_TIMEOUT_S = 1.5;
    private static final double TIP_TAG_LOST_S = 0.4;
    private static final double TIP_TY_SHIFT_DEG = 3.0;

    // Park time estimate = base + distance / speed.
    private static final double PARK_BASE_S = 0.6;
    private static final double PARK_SPEED_IN_PER_S = 40.0;
    // Leg estimate for "flower trip + drive back + fire 4".
    private static final double FLOWER_LEG_EST_S = 9.0;

    // ---- Hardware ----
    private Follower follower;
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final HiveVision vision = new HiveVision();
    private final HiveAimer aimer = new HiveAimer();

    // ---- Field (already converted to this alliance) ----
    private Pose start, startExit, gardenApproach, corridorSouth, corridorNorth, flowerStaging,
            flowerBackoff, flowerPickup, backFire, parkApproach, park;
    private Pose firstCell, secondCell;
    private int[] firstCellTags, secondCellTags;
    private PathChain toGarden, gardenToBack, backToFlower, flowerToBack, backToPark;

    // ---- State ----
    private enum State {
        FIRE_TIP1, CONFIRM_TIP1, FIRE_SPARE, CONFIRM_SPARE,
        TO_GARDEN, SWEEP_GARDEN, TO_BACK_1, FIRE_BACK_1,
        TO_FLOWER, AT_FLOWER, TO_BACK_2, FIRE_BACK_2,
        TO_PARK, PARKED
    }
    private State state;
    private final Timer matchTimer = new Timer();
    private final Timer stateTimer = new Timer();
    private final Timer intakeTailTimer = new Timer();
    private boolean intakeTailRunning = false;

    private Pose targetCell;
    private int heldBalls = PRELOAD_BALLS;
    private int ballsInTargetCell = 0;
    private int tips = 0;
    private int gardenTake = 3;

    // TIP 1 confirmation bookkeeping.
    private double tyBeforeTip;
    private boolean sawTyBeforeTip;
    private double lastSawFirstCellS;
    private String tipNote = "";

    @Override
    public void init() {
        start = at(FieldConstants.RED_START);
        startExit = at(FieldConstants.RED_START_EXIT);
        gardenApproach = at(FieldConstants.RED_GARDEN_APPROACH);
        corridorSouth = at(FieldConstants.RED_CORRIDOR_SOUTH);
        corridorNorth = at(FieldConstants.RED_CORRIDOR_NORTH);
        flowerStaging = at(FieldConstants.RED_FLOWER_STAGING);
        flowerBackoff = at(FieldConstants.RED_FLOWER_BACKOFF);
        flowerPickup = at(FieldConstants.RED_FLOWER_PICKUP);
        backFire = at(FieldConstants.RED_BACK_FIRE);
        parkApproach = at(FieldConstants.RED_PARK_APPROACH);
        park = at(FieldConstants.RED_SHOOTER_PARK);
        firstCell = at(FieldConstants.RED_FIRST_CELL);
        secondCell = at(FieldConstants.RED_SECOND_CELL);
        firstCellTags = FieldConstants.firstCellTags(alliance);
        secondCellTags = FieldConstants.secondCellTags(alliance);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        buildPaths();

        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        vision.init(hardwareMap);
        aimer.init(hardwareMap);

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Ball sensor", shooter.hasBallSensor() ? "found" : "not installed (assuming POLLEN)");
        telemetry.addLine("Place robot touching the audience wall (blue: far wall), facing the upward CELL.");
        telemetry.update();
    }

    @Override
    public void start() {
        matchTimer.resetTimer();
        aimAt(firstCell, firstCellTags);
        shooter.startVolley(BALLS_TO_TIP_FIRST_CELL);
        setState(State.FIRE_TIP1);
    }

    @Override
    public void loop() {
        // Run every cycle, never gated behind a wait.
        follower.update();
        vision.update();
        aimer.update(follower.getPose(), targetCell, vision);
        shooter.update(aimer.getDistanceIn(), isAimedOrGaveUp());
        updateIntakeTail();

        if (PARK_IS_REQUIRED && !isParking() && timeLeft() < parkEstimateS(follower.getPose())) {
            tipNote = "out of time -> parking";
            goPark(true);
        }

        runStateMachine();

        telemetry.addData("State", state);
        telemetry.addData("Time left", "%.1f s", timeLeft());
        telemetry.addData("TIPs", tips);
        telemetry.addData("Held balls (est.)", heldBalls);
        telemetry.addData("Balls in target CELL (est.)", ballsInTargetCell);
        telemetry.addData("Aim", aimer.isUsingVision()
                ? (aimer.isAimedByVision() ? "LOCKED (vision)" : "centering (vision)")
                : "odometry (no tags seen)");
        telemetry.addData("Distance", "%.1f in", aimer.getDistanceIn());
        telemetry.addData("Next ball", shooter.getNextBall());
        telemetry.addData("Flywheel", "%.0f rpm", shooter.getMeasuredRpm());
        telemetry.addData("Note", tipNote);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        aimer.stop();
        vision.close();
    }

    private void runStateMachine() {
        switch (state) {
            case FIRE_TIP1:
                trackFirstCellTags();
                if (shooter.isVolleyDone()) {
                    fired(shooter.getBallsFired());
                    setState(State.CONFIRM_TIP1);
                }
                break;

            case CONFIRM_TIP1:
            case CONFIRM_SPARE:
                handleTip1Confirmation();
                break;

            case FIRE_SPARE:
                if (shooter.isVolleyDone()) {
                    fired(shooter.getBallsFired());
                    setState(State.CONFIRM_SPARE);
                }
                break;

            case TO_GARDEN:
                if (!follower.isBusy()) {
                    intake.in();
                    Pose sweepEnd = at(gardenTake >= 4
                            ? FieldConstants.RED_GARDEN_SWEEP_END_TAKE_4
                            : FieldConstants.RED_GARDEN_SWEEP_END_TAKE_3);
                    PathBuilder sweep = follower.pathBuilder();
                    leg(sweep, gardenApproach, sweepEnd, 1.0);
                    follower.followPath(sweep.build(), SWEEP_MAX_POWER, true);

                    PathBuilder b = follower.pathBuilder();
                    leg(b, sweepEnd, corridorSouth, 0.8);   // turn in the open, off the wall
                    leg(b, corridorSouth, corridorNorth, 1.0);
                    leg(b, corridorNorth, backFire, 1.0);
                    gardenToBack = b.build();
                    setState(State.SWEEP_GARDEN);
                }
                break;

            case SWEEP_GARDEN:
                if (!follower.isBusy()) {
                    heldBalls += gardenTake;
                    startIntakeTail();
                    follower.followPath(gardenToBack, true);
                    setState(State.TO_BACK_1);
                }
                break;

            case TO_BACK_1:
                shooter.preSpin(FieldConstants.distanceToHive(backFire, secondCell));
                if (!follower.isBusy()) {
                    shooter.startVolley(heldBalls);
                    setState(State.FIRE_BACK_1);
                }
                break;

            case FIRE_BACK_1:
                if (shooter.isVolleyDone()) {
                    fired(shooter.getBallsFired());
                    double needed = FLOWER_LEG_EST_S + (PARK_IS_REQUIRED ? parkEstimateS(backFire) : 0);
                    if (timeLeft() > needed) {
                        follower.followPath(backToFlower, true);
                        setState(State.TO_FLOWER);
                    } else {
                        tipNote = "no time for the FLOWER -> parking";
                        goPark(false);
                    }
                }
                break;

            case TO_FLOWER:
                if (!follower.isBusy()) {
                    intake.in();
                    setState(State.AT_FLOWER);
                }
                break;

            case AT_FLOWER:
                if (stateTimer.getElapsedTimeSeconds() > FLOWER_DWELL_S) {
                    heldBalls = Math.min(MAX_HELD_BALLS, heldBalls + 4);
                    startIntakeTail();
                    follower.followPath(flowerToBack, true);
                    setState(State.TO_BACK_2);
                }
                break;

            case TO_BACK_2:
                shooter.preSpin(FieldConstants.distanceToHive(backFire, secondCell));
                if (!follower.isBusy()) {
                    shooter.startVolley(heldBalls);
                    setState(State.FIRE_BACK_2);
                }
                break;

            case FIRE_BACK_2:
                if (shooter.isVolleyDone()) {
                    fired(shooter.getBallsFired());
                    goPark(false);
                }
                break;

            case TO_PARK:
                if (!follower.isBusy()) {
                    shooter.stop();
                    intake.stop();
                    setState(State.PARKED);
                }
                break;

            case PARKED:
                break;
        }
    }

    // ---- TIP 1 ----

    // While firing at the first CELL, remember where its tags sit (ty) and
    // when they were last seen, to tell afterward whether it tipped.
    private void trackFirstCellTags() {
        if (vision.isTargetVisible()) {
            tyBeforeTip = vision.getTargetTyDeg();
            sawTyBeforeTip = true;
            lastSawFirstCellS = matchTimer.getElapsedTimeSeconds();
        }
    }

    // After the TIP 1 volley: TIPPED if the second CELL's tags show up, or
    // the first CELL's tags vanish / jump. NOT TIPPED if, after the timeout,
    // the first CELL's tags are still right where they were -> fire the
    // spare 4th ball at it (once). Unknown -> assume it tipped and keep the
    // ball (never fire at a CELL that may now be facing down).
    private void handleTip1Confirmation() {
        double now = matchTimer.getElapsedTimeSeconds();
        boolean firstVisible = vision.isTargetVisible();
        if (firstVisible) lastSawFirstCellS = now;

        boolean tipped = vision.seesAny(secondCellTags)
                || (sawTyBeforeTip && now - lastSawFirstCellS > TIP_TAG_LOST_S)
                || (firstVisible && sawTyBeforeTip
                    && Math.abs(vision.getTargetTyDeg() - tyBeforeTip) > TIP_TY_SHIFT_DEG);

        if (tipped) {
            tipNote = "TIP 1 seen";
            onTip1Done(true);
            return;
        }
        if (stateTimer.getElapsedTimeSeconds() < TIP_CONFIRM_TIMEOUT_S) return;

        boolean stillUp = firstVisible && sawTyBeforeTip
                && Math.abs(vision.getTargetTyDeg() - tyBeforeTip) <= TIP_TY_SHIFT_DEG;
        if (state == State.CONFIRM_TIP1 && stillUp && heldBalls > 0) {
            tipNote = "no TIP 1 yet -> firing spare";
            shooter.startVolley(1);
            setState(State.FIRE_SPARE);
        } else {
            // stillUp here means even the spare missed: carry on with the
            // plan anyway (the TIP may still come in TELEOP).
            tipNote = stillUp ? "TIP 1 MISSED, continuing" : "TIP 1 assumed (couldn't tell)";
            onTip1Done(!stillUp);
        }
    }

    private void onTip1Done(boolean countTip) {
        if (countTip) tips++;
        aimAt(secondCell, secondCellTags);
        gardenTake = Math.min(MAX_HELD_BALLS - heldBalls, 4);
        follower.followPath(toGarden, true);
        setState(State.TO_GARDEN);
    }

    // ---- Helpers ----

    private void fired(int balls) {
        heldBalls = Math.max(0, heldBalls - balls);
        ballsInTargetCell += balls;
        if (targetCell == secondCell && ballsInTargetCell >= BALLS_TO_TIP_EMPTY_CELL) {
            tips++;
            tipNote = "TIP 2 expected";
        }
    }

    private void aimAt(Pose cell, int[] tags) {
        targetCell = cell;
        vision.setTargetTags(tags);
        ballsInTargetCell = 0;
    }

    private boolean isAimedOrGaveUp() {
        if (aimer.isAimedByVision()) return true;
        double waited = stateTimer.getElapsedTimeSeconds();
        if (waited > AIM_GIVE_UP_S) return true;
        return waited > AIM_FALLBACK_S && aimer.isAimedByOdometry();
    }

    private void goPark(boolean emergency) {
        shooter.abortVolley();
        follower.followPath(emergency ? emergencyParkPath() : backToPark, true);
        intake.stop();
        setState(State.TO_PARK);
    }

    // Route to park from wherever the robot is, using the same checked
    // waypoints so it never turns near a wall/FLOWER or cuts through the
    // partner's half of the LOADING ZONE. Regions are tested in red
    // coordinates (forAlliance is its own inverse).
    private PathChain emergencyParkPath() {
        Pose here = follower.getPose();
        Pose red = at(here);
        List<Pose> route = new ArrayList<>();
        route.add(here);
        if (red.getY() < 40) {                       // GARDEN / start area
            route.add(corridorSouth);
            route.add(corridorNorth);
        } else if (red.getX() < 28) {                // at the FLOWER
            route.add(flowerBackoff);
            route.add(flowerStaging);
            route.add(corridorNorth);
        } else if (red.getY() < FieldConstants.RED_CORRIDOR_NORTH.getY()) {  // in the corridor
            route.add(corridorNorth);
        }
        route.add(parkApproach);
        route.add(park);

        PathBuilder b = follower.pathBuilder();
        Pose from = route.get(0);
        for (int i = 1; i < route.size(); i++) {
            Pose to = route.get(i);
            if (Math.hypot(to.getX() - from.getX(), to.getY() - from.getY()) < 1.0) continue;
            leg(b, from, to, (i == 1) ? 0.8 : 1.0);
            from = to;
        }
        return b.build();
    }

    private boolean isParking() {
        return state == State.TO_PARK || state == State.PARKED;
    }

    private double parkEstimateS(Pose from) {
        return PARK_BASE_S + Math.hypot(park.getX() - from.getX(), park.getY() - from.getY()) / PARK_SPEED_IN_PER_S;
    }

    private double timeLeft() {
        return AUTO_LENGTH_S - matchTimer.getElapsedTimeSeconds();
    }

    private void startIntakeTail() {
        intakeTailRunning = true;
        intakeTailTimer.resetTimer();
    }

    private void updateIntakeTail() {
        if (intakeTailRunning && intakeTailTimer.getElapsedTimeSeconds() > INTAKE_TAIL_S) {
            intake.stop();
            intakeTailRunning = false;
        }
    }

    private void setState(State s) {
        state = s;
        stateTimer.resetTimer();
    }

    private Pose at(Pose redPose) {
        return FieldConstants.forAlliance(redPose, alliance);
    }

    // Routes follow FieldConstants' checked waypoints; turns only happen on
    // the legs with a turnEndT < 1 or between differently-headed waypoints
    // in open floor (see the ROUTE WAYPOINTS note in FieldConstants).
    private void buildPaths() {
        PathBuilder b = follower.pathBuilder();
        leg(b, start, startExit, 1.0);                // straight off the wall
        leg(b, startExit, gardenApproach, 0.6);       // finish turning before the wall
        toGarden = b.build();

        b = follower.pathBuilder();
        leg(b, backFire, corridorNorth, 1.0);
        leg(b, corridorNorth, flowerStaging, 1.0);
        leg(b, flowerStaging, flowerBackoff, 1.0);    // turn to face the FLOWER
        leg(b, flowerBackoff, flowerPickup, 1.0);
        backToFlower = b.build();

        b = follower.pathBuilder();
        leg(b, flowerPickup, flowerBackoff, 1.0);     // back straight out first
        leg(b, flowerBackoff, flowerStaging, 1.0);
        leg(b, flowerStaging, corridorNorth, 1.0);
        leg(b, corridorNorth, backFire, 1.0);
        flowerToBack = b.build();

        b = follower.pathBuilder();
        leg(b, backFire, park, 1.0);
        backToPark = b.build();
        // gardenToBack is built when the sweep starts (its start depends on
        // whether the sweep takes 3 or 4 balls).
    }

    // One straight leg. Heading turns linearly from `from`'s heading to
    // `to`'s, finishing at fraction turnEndT of the leg (1.0 = spread over
    // the whole leg); constant if the two headings match.
    private static void leg(PathBuilder b, Pose from, Pose to, double turnEndT) {
        b.addPath(new BezierLine(from, to));
        if (Math.abs(Math.sin((to.getHeading() - from.getHeading()) / 2)) < 1e-6) {
            b.setConstantHeadingInterpolation(to.getHeading());
        } else if (turnEndT < 1.0) {
            b.setLinearHeadingInterpolation(from.getHeading(), to.getHeading(), turnEndT);
        } else {
            b.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
        }
    }
}
