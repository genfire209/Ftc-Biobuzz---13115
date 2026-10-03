package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// PARK-ONLY AUTO, for when this robot is the alliance's parker and the
// partner runs the two-tip auto. Earns LEAVE (3) + AUTO PARK (5).
//
// Starts touching the alliance wall just on the audience side of the
// LOADING ZONE (red; blue rotated 180 deg), moves 2in off the wall so
// LEAVE counts, and parks in the zone half nearest it, leaving the other
// half for the shooting robot. Holds its 4 preloaded POLLEN for TELEOP.
public abstract class ParkOnlyAutoBase extends OpMode {

    protected FieldConstants.Alliance alliance;

    private Follower follower;
    private PathChain toPark;

    @Override
    public void init() {
        Pose start = FieldConstants.forAlliance(FieldConstants.RED_PARKER_START, alliance);
        Pose park = FieldConstants.forAlliance(FieldConstants.RED_PARKER_PARK, alliance);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(start, park))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();

        telemetry.addData("Alliance", alliance);
        telemetry.addLine("Place robot touching the alliance wall, just outside the LOADING ZONE.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(toPark, true);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("Parked", !follower.isBusy());
        telemetry.update();
    }
}
