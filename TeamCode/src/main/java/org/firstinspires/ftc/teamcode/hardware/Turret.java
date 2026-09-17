package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

// Rotates the Limelight independently of the chassis so it can point at
// the HIVE regardless of robot heading. Closed-loop angle control via
// encoder ticks + RUN_TO_POSITION -- non-blocking, since RUN_TO_POSITION
// just issues a target and returns; the motor's own controller drives
// toward it asynchronously (same pattern as the sibling season's
// Robot.moveRobotInches).
//
// Hardware not built yet: motor name, TICKS_PER_DEGREE (depends on motor
// encoder resolution + turret gear ratio), and the rotation limits are
// all placeholders -- TODO: MEASURE once the turret exists.
public class Turret {

    // TODO: DECIDE -- device name, must match Driver Hub config.
    private static final String MOTOR_NAME = "turret";

    // TODO: MEASURE -- encoder ticks per degree of turret rotation
    // (motor encoder resolution combined with the turret's gear ratio).
    private static final double TICKS_PER_DEGREE = 10.0;

    // TODO: MEASURE -- mechanical rotation limits, to avoid winding up
    // wiring. Angle 0 is assumed to be "pointing straight forward",
    // aligned with the robot chassis heading -- verify this assumption
    // once the turret is mounted and re-zero if it isn't centered.
    private static final double MIN_ANGLE_DEG = -160.0;
    private static final double MAX_ANGLE_DEG = 160.0;

    // TODO: TUNE -- how hard the turret drives toward its target.
    private static final double POWER = 0.5;

    private DcMotorEx motor;

    public void init(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Commands the turret toward angleDeg, relative to the robot chassis
    // (0 = straight forward, positive = counterclockwise viewed from
    // above -- matches Pedro's heading convention). Non-blocking.
    public void setTargetAngleDeg(double angleDeg) {
        double clamped = Range.clip(angleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        int ticks = (int) Math.round(clamped * TICKS_PER_DEGREE);
        motor.setTargetPosition(ticks);
        motor.setPower(POWER);
    }

    public double getCurrentAngleDeg() {
        return motor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public boolean isAtTarget() {
        return !motor.isBusy();
    }

    public void stop() {
        motor.setPower(0);
    }
}
