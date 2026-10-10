package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// goBILDA Pinpoint V2 + two SWYFT odometry pods on the meet-1 robot.
// Pose is in field inches/radians, in the frame FieldConstants uses
// (origin = red/audience corner, +x toward blue, +y toward the far wall,
// heading 0 = facing +x, counterclockwise positive).
//
// Call update() once per loop before reading the pose.
public class Odometry {

    private static final String PINPOINT_NAME = "pinpoint";

    // Bench-measured 2026-10-08: X pod 33.8, Y pod 34.1 counts/mm (4096
    // counts per turn of the 38 mm wheel).
    private static final double COUNTS_PER_MM = 34.0;

    // From the CAD (Ri3D Robot - Belt Drive.step), robot center = middle of
    // the four wheel axles. goBILDA's signs: the forward (X) pod's offset is
    // + when LEFT of center; the strafe (Y) pod's offset is + when FORWARD
    // of center. TODO: VERIFY with a ruler once the pods are mounted.
    private static final double X_POD_LEFT_OF_CENTER_MM = 48.5;
    private static final double Y_POD_FORWARD_OF_CENTER_MM = -135.6;

    // TODO: VERIFY with the push test on the auto's INIT screen: pushing the
    // robot toward its intake must read "forward +", pushing it to its left
    // must read "left +". Flip the one that's wrong.
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private GoBildaPinpointDriver pinpoint;

    // Returns false (and the robot runs without odometry) if the Pinpoint
    // isn't in the robot config.
    public boolean init(HardwareMap hw) {
        pinpoint = hw.tryGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
        if (pinpoint == null) return false;

        pinpoint.setOffsets(X_POD_LEFT_OF_CENTER_MM, Y_POD_FORWARD_OF_CENTER_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(COUNTS_PER_MM, DistanceUnit.MM);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        // Calibrates the gyro: the robot must sit still for about a second.
        pinpoint.resetPosAndIMU();
        return true;
    }

    public boolean isPresent() { return pinpoint != null; }

    // Configured and reporting READY (not calibrating, no pod/IMU fault).
    // Call update() first -- the status comes with each read.
    public boolean isReady() {
        return pinpoint != null && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public void setPose(double xIn, double yIn, double headingRad) {
        if (pinpoint == null) return;
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.RADIANS, headingRad));
    }

    public void update() {
        if (pinpoint != null) pinpoint.update();
    }

    public double getX() { return pinpoint == null ? 0 : pinpoint.getPosX(DistanceUnit.INCH); }

    public double getY() { return pinpoint == null ? 0 : pinpoint.getPosY(DistanceUnit.INCH); }

    public double getHeading() { return pinpoint == null ? 0 : pinpoint.getHeading(AngleUnit.RADIANS); }

    public String getStatus() {
        return pinpoint == null ? "NOT CONFIGURED" : pinpoint.getDeviceStatus().toString();
    }
}
