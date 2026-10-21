package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// The Nemesis turret: shooter + Limelight on a 90-tooth ring, turned by TWO
// SWYFT Speed servos, each with a 60-tooth gear on the ring (the turret
// turns 2/3 as far as the servos).
//   - "turret_left"  (Control Hub servo port 2): the servo on the robot's
//     LEFT side of the turret.
//   - "turret_right" (Expansion Hub servo port 0): the one on the RIGHT.
//     On the other hub so the two don't share a servo power supply.
// Both get the same position, so they must agree on where "front" is or
// they fight each other (buzzing, hot servos, stripped printed gears). The
// right servo has a small saved TRIM to line it up with the left one.
//
// Angles are turret degrees from FRONT (0 = shooter facing the intake
// side), positive = turning toward the robot's left (counterclockwise seen
// from above), like Pedro headings. The turret only turns between the two
// saved limits -- it doesn't need 360, and the wires shouldn't wind up.
//
// Every number is set on the robot with Nemesis Setup Test (TURRET + TRIM
// pages) and saved on the hub; init() loads them. Servos have no position
// feedback, so "where it is" means "where it was last told to go".
//
// Optional: without the servos in the config, hasServos() is false and
// everything is a no-op, so OpModes still run on a robot without a turret.
public class Turret {

    public static final String LEFT_SERVO_NAME = "turret_left";
    public static final String RIGHT_SERVO_NAME = "turret_right";

    // Saved on the hub by Nemesis Setup Test.
    public static final String FRONT_FILE = "nemesis_turret_front";        // servo position, facing front
    public static final String LEFT_90_FILE = "nemesis_turret_left90";     // servo position, facing the robot's left
    public static final String LEFT_LIMIT_FILE = "nemesis_turret_left_limit_deg";
    public static final String RIGHT_LIMIT_FILE = "nemesis_turret_right_limit_deg";
    public static final String RIGHT_TRIM_FILE = "nemesis_turret_right_trim";

    // Defaults until the setup test saves real numbers. A servo position
    // step per turret degree of 0.005 assumes ~300 deg of servo travel over
    // 0..1 and the 60:90 gearing (1.5 servo deg per turret deg).
    private static final double DEFAULT_FRONT = 0.5;
    private static final double DEFAULT_POSITION_PER_DEG = 0.005;
    private static final double DEFAULT_LEFT_LIMIT_DEG = 90.0;
    private static final double DEFAULT_RIGHT_LIMIT_DEG = -90.0;

    // TODO: VERIFY: both servos are mounted the same way (CAD), so the same
    // position turns both the same way. If one fights the other at every
    // angle, not just near front, set this true.
    private static final boolean RIGHT_SERVO_REVERSED = false;

    // For isAtTarget() only (no feedback): a conservative turning speed.
    // TODO: TUNE once it turns on the robot.
    private static final double TURRET_DEG_PER_S = 250.0;
    private static final double SETTLE_S = 0.15;

    private Servo left;
    private Servo right;

    private double frontPosition = DEFAULT_FRONT;
    private double positionPerDeg = DEFAULT_POSITION_PER_DEG;
    private double leftLimitDeg = DEFAULT_LEFT_LIMIT_DEG;
    private double rightLimitDeg = DEFAULT_RIGHT_LIMIT_DEG;
    private double rightTrim = 0;

    private double targetDeg = 0;
    private boolean commanded = false;
    private double moveTimeS = 0;
    private final ElapsedTime moveTimer = new ElapsedTime();

    // Finds the servos and loads the saved numbers. Doesn't move anything:
    // call goFront() (every OpMode does at INIT, so the turret always starts
    // facing front -- R103 allows servos to move during INIT).
    public void init(HardwareMap hw) {
        left = hw.tryGet(Servo.class, LEFT_SERVO_NAME);
        right = hw.tryGet(Servo.class, RIGHT_SERVO_NAME);
        if (right != null && RIGHT_SERVO_REVERSED) right.setDirection(Servo.Direction.REVERSE);
        loadCalibration();
    }

    public void loadCalibration() {
        frontPosition = SavedNumber.load(FRONT_FILE, DEFAULT_FRONT);
        double left90 = SavedNumber.load(LEFT_90_FILE, DEFAULT_FRONT + 90 * DEFAULT_POSITION_PER_DEG);
        positionPerDeg = (left90 - frontPosition) / 90.0;
        if (Math.abs(positionPerDeg) < 0.0005) positionPerDeg = DEFAULT_POSITION_PER_DEG;   // bad save
        leftLimitDeg = Math.max(0, SavedNumber.load(LEFT_LIMIT_FILE, DEFAULT_LEFT_LIMIT_DEG));
        rightLimitDeg = Math.min(0, SavedNumber.load(RIGHT_LIMIT_FILE, DEFAULT_RIGHT_LIMIT_DEG));
        rightTrim = SavedNumber.load(RIGHT_TRIM_FILE, 0);
    }

    public boolean hasServos() { return left != null || right != null; }

    // Both servos found (one alone can turn it, but slower and weaker).
    public boolean hasBothServos() { return left != null && right != null; }

    public void goFront() { setTargetAngleDeg(0); }

    // Turns to angleDeg (0 = front, + = toward the robot's left), clipped to
    // the saved limits. Returns right away; the servos get there on their own.
    public void setTargetAngleDeg(double angleDeg) {
        double clipped = Range.clip(angleDeg, rightLimitDeg, leftLimitDeg);
        // First move: the turret could be anywhere (servos were off).
        double fromDeg = commanded ? targetDeg : (clipped >= 0 ? rightLimitDeg : leftLimitDeg);
        commanded = true;
        moveTimeS = Math.abs(clipped - fromDeg) / TURRET_DEG_PER_S + SETTLE_S;
        moveTimer.reset();
        targetDeg = clipped;
        setPosition(frontPosition + clipped * positionPerDeg);
    }

    // Last commanded angle (no feedback).
    public double getCurrentAngleDeg() { return targetDeg; }

    // Probably there by now: long enough has passed for the last move.
    public boolean isAtTarget() { return moveTimer.seconds() > moveTimeS; }

    // The turret is held by its servos, so "stop" just means stay put.
    public void stop() {}

    public boolean isAtLeftLimit() { return targetDeg >= leftLimitDeg - 0.01; }

    public boolean isAtRightLimit() { return targetDeg <= rightLimitDeg + 0.01; }

    public double getLeftLimitDeg() { return leftLimitDeg; }

    public double getRightLimitDeg() { return rightLimitDeg; }

    // ---- For Nemesis Setup Test ----

    // Sends a raw position (left servo; right gets it + trim), no limits.
    public void setPosition(double position) {
        double p = Range.clip(position, 0, 1);
        if (left != null) left.setPosition(p);
        if (right != null) right.setPosition(Range.clip(p + rightTrim, 0, 1));
    }

    // Turret angle a raw position would give, with the saved calibration.
    public double angleOf(double position) { return (position - frontPosition) / positionPerDeg; }

    public void setRightTrim(double trim) {
        rightTrim = trim;
    }

    public double getRightTrim() { return rightTrim; }

    public double getFrontPosition() { return frontPosition; }

    public double getPositionPerDeg() { return positionPerDeg; }
}
