package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Launcher;
import org.firstinspires.ftc.teamcode.hardware.ShotCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.HiveAimer;
import org.firstinspires.ftc.teamcode.vision.HiveRangeFinder;

// Shared autonomous state machine: drive to a firing position via Pedro
// Pathing, spin the flywheel to a live distance-computed RPM, fire the 4
// preloaded POLLEN (non-blocking -- follower.update() and launcher.update()
// run every loop() no matter what state the fire sequence is in), then
// drive to LEAVE/PARK.
//
// Distance source: the HIVE's own AprilTag cluster (HiveRangeFinder) is
// primary -- it measures the actual physical distance to the target,
// immune to the manual's stated +/-1in field-to-field tolerance and to
// Pinpoint drift. Pedro odometry (FieldConstants.distanceToHive) is the
// fallback whenever no tag is visible that cycle.
//
// Subclasses set startPose/firingPose/parkPose/hiveTarget/alliance and are
// @Autonomous-annotated, no-arg-constructor menu entries -- see RedAuto.java
// / BlueAuto.java for examples. Concrete starting-tile poses below are
// placeholders (TODO: MEASURE) since AUTO starting position(s) aren't
// decided yet.
public abstract class HiveShootAutoBase extends OpMode {

    // TODO: DECIDE/MEASURE -- feeder device name, once the launcher/intake
    // mechanism exists.
    private static final String FEEDER_MOTOR_NAME = "feeder";
    private static final double FEEDER_POWER = 0.6;
    private static final double FEED_DURATION_S = 1.0;

    protected Pose startPose;
    protected Pose firingPose;
    protected Pose parkPose;
    protected Pose hiveTarget;
    protected FieldConstants.Alliance alliance;

    private Follower follower;
    private final Launcher launcher = new Launcher();
    private final HiveRangeFinder rangeFinder = new HiveRangeFinder();
    private final HiveAimer aimer = new HiveAimer();
    private DcMotor feeder;

    private PathChain toFiringPosition;
    private PathChain toPark;

    private int pathState;
    private FireState fireState = FireState.IDLE;
    private final Timer fireTimer = new Timer();

    private enum FireState { IDLE, SPIN_UP, FEEDING, DONE }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        toFiringPosition = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firingPose))
                .setTangentHeadingInterpolation()
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(firingPose, parkPose))
                .setTangentHeadingInterpolation()
                .build();

        launcher.init(hardwareMap);
        rangeFinder.init(hardwareMap, alliance);
        aimer.init(hardwareMap);
        feeder = hardwareMap.get(DcMotor.class, FEEDER_MOTOR_NAME);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        // Must run every cycle, unconditionally -- never gated behind a
        // blocking wait, or the robot stops moving/spinning up mid-shot.
        follower.update();
        launcher.update();
        aimer.update(follower.getPose(), hiveTarget);

        autonomousPathUpdate();
        updateFireSequence();

        Double tagRangeIn = rangeFinder.getRangeIn();
        telemetry.addData("path state", pathState);
        telemetry.addData("fire state", fireState);
        telemetry.addData("distance source", (tagRangeIn != null) ? "AprilTag" : "odometry (no tag seen)");
        telemetry.addData("distance to hive (in)", "%.1f", distanceToHiveIn(tagRangeIn));
        telemetry.addData("launcher target rpm", "%.0f", launcherTargetRpm(tagRangeIn));
        telemetry.addData("launcher measured rpm", "%.0f", launcher.getMeasuredRpm());
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.stop();
        feeder.setPower(0);
        rangeFinder.close();
        aimer.stop();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(toFiringPosition, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    fireState = FireState.SPIN_UP;
                    setPathState(2);
                }
                break;
            case 2:
                if (fireState == FireState.DONE) {
                    follower.followPath(toPark);
                    setPathState(3);
                }
                break;
            case 3:
                // Driving to LEAVE/PARK; nothing further to sequence.
                break;
        }
    }

    private void updateFireSequence() {
        switch (fireState) {
            case SPIN_UP:
                launcher.setTargetRpm(launcherTargetRpm(rangeFinder.getRangeIn()));
                if (launcher.isReadyToFire()) {
                    feeder.setPower(FEEDER_POWER);
                    fireTimer.resetTimer();
                    fireState = FireState.FEEDING;
                }
                break;
            case FEEDING:
                if (fireTimer.getElapsedTimeSeconds() > FEED_DURATION_S) {
                    feeder.setPower(0);
                    launcher.stop();
                    fireState = FireState.DONE;
                }
                break;
            case IDLE:
            case DONE:
                break;
        }
    }

    // Prefers the AprilTag's measured range to the HIVE (actual physical
    // distance to the target); falls back to Pedro odometry distance when
    // no tag is visible this cycle.
    private double distanceToHiveIn(Double tagRangeIn) {
        return (tagRangeIn != null) ? tagRangeIn : FieldConstants.distanceToHive(follower.getPose(), hiveTarget);
    }

    private double launcherTargetRpm(Double tagRangeIn) {
        return ShotCalculator.getTargetRpm(distanceToHiveIn(tagRangeIn));
    }

    private void setPathState(int state) {
        pathState = state;
    }
}
