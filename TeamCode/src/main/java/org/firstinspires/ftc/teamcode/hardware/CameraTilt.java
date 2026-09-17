package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// Servo controlling the Limelight's up/down pitch, so it can look up
// under the HIVE's downward-facing AprilTag cluster (see
// vision/HiveRangeFinder.java) even though the CELL sits well above
// robot height.
//
// Hardware not built yet: servo name and the angle<->position mapping
// are placeholders -- TODO: MEASURE once the tilt mount exists (point
// the camera at a couple of known angles, record the servo positions
// that produced them, and fit the constants below).
public class CameraTilt {

    // TODO: DECIDE -- device name, must match Driver Hub config.
    private static final String SERVO_NAME = "camera_tilt";

    // TODO: MEASURE -- servo position (0.0-1.0) at each tilt extreme, and
    // the actual camera pitch angle (degrees) each one corresponds to.
    private static final double POSITION_AT_MIN_ANGLE = 0.0;
    private static final double POSITION_AT_MAX_ANGLE = 1.0;
    private static final double MIN_ANGLE_DEG = -10.0;
    private static final double MAX_ANGLE_DEG = 60.0;

    private Servo servo;

    public void init(HardwareMap hw) {
        servo = hw.get(Servo.class, SERVO_NAME);
    }

    // angleDeg: camera pitch, positive = tilted upward.
    public void setTargetAngleDeg(double angleDeg) {
        double clampedAngle = Range.clip(angleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        double t = (clampedAngle - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
        double position = POSITION_AT_MIN_ANGLE + t * (POSITION_AT_MAX_ANGLE - POSITION_AT_MIN_ANGLE);
        servo.setPosition(Range.clip(position, 0.0, 1.0));
    }
}
