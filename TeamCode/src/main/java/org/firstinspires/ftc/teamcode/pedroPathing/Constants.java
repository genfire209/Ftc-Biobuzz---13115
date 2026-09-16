package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Drivetrain/localizer/follower setup for Pedro Pathing. Motor names match
// BasicTeleOp. Everything marked "TODO: MEASURE" is a placeholder that must
// be re-measured/re-tuned on the real robot (Pedro's own tuning OpModes,
// once a goBILDA Pinpoint is physically mounted) — do not trust these
// numbers, they are structurally-correct placeholders only.
public class Constants {

    // TODO: MEASURE — robot mass in kg, changes once the launcher is added.
    public static double MASS = 10.0;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("front_left_drive")
            .rightFrontMotorName("front_right_drive")
            .leftRearMotorName("back_left_drive")
            .rightRearMotorName("back_right_drive")
            // TODO: MEASURE — re-verify these directions by driving under Pedro;
            // do not assume BasicTeleOp's raw DcMotor.Direction.REVERSE carries over.
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            // TODO: MEASURE — via Pedro's ForwardVelocityTuner / LateralVelocityTuner.
            .xVelocity(60.0)
            .yVelocity(50.0);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(MASS)
            // TODO: MEASURE — via Pedro's ForwardZeroPowerAccelerationTuner /
            // LateralZeroPowerAccelerationTuner.
            .forwardZeroPowerAcceleration(-30.0)
            .lateralZeroPowerAcceleration(-50.0);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            // TODO: MEASURE — physical offset (inches) of the dead-wheel pods from
            // robot center, once a goBILDA Pinpoint is mounted.
            .forwardPodY(0.0)
            .strafePodX(0.0)
            .distanceUnit(DistanceUnit.INCH)
            // TODO: MEASURE — must match the Pinpoint's name in the Driver Hub config.
            .hardwareMapName("pinpoint")
            // TODO: MEASURE — depends on which goBILDA pod model is purchased.
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // TODO: MEASURE — bench-verify sign by pushing the robot and checking
            // reported X/Y.
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
