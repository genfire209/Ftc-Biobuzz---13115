package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.BallSensor;

// Bench test for the REV Color Sensor V3 at the flywheel feed slot.
// Hold a POLLEN, a red NECTAR, a blue NECTAR, and nothing in the slot one
// at a time and note the hue and distance each shows, then tighten the
// ranges in hardware/BallSensor.java so each one reads correctly.
@TeleOp(name = "Ball Sensor Test", group = "systemTest")
public class BallSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        BallSensor sensor = new BallSensor();
        sensor.init(hardwareMap);

        if (!sensor.isPresent()) {
            telemetry.addLine("No 'ball_sensor' in the robot configuration.");
            telemetry.addLine("Add it as an I2C device of type 'REV Color Sensor V3'.");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("Ready. Press start, then hold each ball in the slot.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Reads as", sensor.read());
            telemetry.addData("Hue (deg)", "%.0f", sensor.getHue());
            telemetry.addData("Distance (cm)", "%.1f", sensor.getDistanceCm());
            telemetry.update();
        }
    }
}
