package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Intake roller. Hardware not built yet: motor name, direction and power
// are placeholders -- TODO: MEASURE/DECIDE once the intake exists.
//
// G407: the robot may not control more than 4 SCORING ELEMENTS at once.
// The auto relies on the GARDEN sweep distance (FieldConstants) to take
// the right number of balls -- check on the real robot that the intake
// can't swallow a 5th.
public class Intake {

    private static final String MOTOR_NAME = "intake";
    private static final double IN_POWER = 1.0;

    private DcMotor motor;

    public void init(HardwareMap hw) {
        motor = hw.get(DcMotor.class, MOTOR_NAME);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void in() {
        motor.setPower(IN_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }
}
