package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// Servo controlling the Limelight's up/down pitch, so it can look up
// under the HIVE's downward-facing AprilTag cluster (see
// vision/HiveRangeFinder.java) even though the CELL sits well above
// robot height.
//
// On the Nemesis robot: a goBILDA servo (Control Hub servo port 0) on the
// printed tilt mount from the belt-drive robot. For now the Limelight just
// holds one saved tilt (holdSavedPosition), set on the Nemesis Setup Test
// TILT page so the HIVE tags are in view from where we shoot.
//
// The angle<->position mapping below is still a placeholder (only
// HiveAimer uses it) -- TODO: MEASURE: point the camera at a couple of
// known angles, record the servo positions, and fit the constants below.
//
// Optional: without the servo in the config, hasServo() is false and the
// rest does nothing.
public class CameraTilt {

    // TODO: DECIDE -- device name, must match Driver Hub config.
    private static final String SERVO_NAME = "camera_tilt";

    public static final String SAVED_POSITION_FILE = "nemesis_ll_tilt";
    private static final double DEFAULT_POSITION = 0.5;

    // TODO: MEASURE -- servo position (0.0-1.0) at each tilt extreme, and
    // the actual camera pitch angle (degrees) each one corresponds to.
    private static final double POSITION_AT_MIN_ANGLE = 0.0;
    private static final double POSITION_AT_MAX_ANGLE = 1.0;
    private static final double MIN_ANGLE_DEG = -10.0;
    private static final double MAX_ANGLE_DEG = 60.0;

    private Servo servo;
    private double targetAngleDeg = 0.0;

    public void init(HardwareMap hw) {
        servo = hw.tryGet(Servo.class, SERVO_NAME);
    }

    public boolean hasServo() { return servo != null; }

    public static double savedPosition() {
        return Range.clip(SavedNumber.load(SAVED_POSITION_FILE, DEFAULT_POSITION), 0, 1);
    }

    // The tilt saved on the hub by Nemesis Setup Test.
    public void holdSavedPosition() {
        setPosition(savedPosition());
    }

    public void setPosition(double position) {
        if (servo != null) servo.setPosition(Range.clip(position, 0, 1));
    }

    // Last commanded pitch (degrees). The servo has no position feedback,
    // so this assumes it got there -- HiveAimer's ty distance math uses it.
    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    // angleDeg: camera pitch, positive = tilted upward.
    public void setTargetAngleDeg(double angleDeg) {
        double clampedAngle = Range.clip(angleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        targetAngleDeg = clampedAngle;
        double t = (clampedAngle - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
        double position = POSITION_AT_MIN_ANGLE + t * (POSITION_AT_MAX_ANGLE - POSITION_AT_MIN_ANGLE);
        if (servo != null) servo.setPosition(Range.clip(position, 0.0, 1.0));
    }
}
