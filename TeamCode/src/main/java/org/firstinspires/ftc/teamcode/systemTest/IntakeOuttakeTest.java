package org.firstinspires.ftc.teamcode.systemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.BallPath;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;

// Runs the two meet-1 ball motors by hand (raw power, no speed control):
//   "intake"           Expansion Hub motor port 3 -- front wheels + ramp
//   "launcher_pollen"  Expansion Hub motor port 0 -- outtake flywheel
//                      (Matrix 12V; its encoder cable in Expansion Hub
//                      encoder port 0)
//
//   R2 hold              intake IN (pulls balls in and up the ramp)
//   L2 hold              intake OUT (spits them back out the front)
//   Cross                outtake on / off
//   D-pad up / down      outtake power +/-5%
//   Circle               gate open / shut (leaves it alone until pressed)
//   L1                   reset the highest-seen outtake speed
//
// Uses the same motor directions as Meet 1 TeleOp (Flywheel.DIRECTION,
// BallPath.INTAKE_DIRECTION), so a motor spinning the wrong way here gets
// fixed for both in one place.
@TeleOp(name = "Intake + Outtake Test", group = "systemTest")
public class IntakeOuttakeTest extends LinearOpMode {

    private static final double START_OUTTAKE_POWER = 0.50;
    private static final double POWER_STEP = 0.05;
    private static final double TRIGGER_PRESSED = 0.5;
    // Outtake on this long with a 0 reading -> the encoder isn't plugged in.
    private static final double ENCODER_CHECK_S = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.get(DcMotor.class, BallPath.INTAKE_NAME);
        intake.setDirection(BallPath.INTAKE_DIRECTION);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, Flywheel.MOTOR_NAME);
        outtake.setDirection(Flywheel.DIRECTION);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Servo gate = hardwareMap.tryGet(Servo.class, "shooter_gate");

        double outtakePower = START_OUTTAKE_POWER;
        boolean outtakeOn = false;
        double maxTicksPerSec = 0;
        String gateState = gate == null ? "not in config" : "not moved yet";
        boolean gateOpen = false;
        ElapsedTime outtakeOnTimer = new ElapsedTime();

        telemetry.addLine("Ready. R2 intake in, L2 intake out, Cross outtake on/off.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            String intakeState;
            if (gamepad1.right_trigger > TRIGGER_PRESSED) {
                intake.setPower(BallPath.INTAKE_POWER);
                intakeState = "IN";
            } else if (gamepad1.left_trigger > TRIGGER_PRESSED) {
                intake.setPower(BallPath.SPIT_POWER);
                intakeState = "OUT";
            } else {
                intake.setPower(0);
                intakeState = "stopped";
            }

            if (gamepad1.crossWasPressed()) {
                outtakeOn = !outtakeOn;
                outtakeOnTimer.reset();
            }
            if (gamepad1.dpadUpWasPressed()) outtakePower = Range.clip(outtakePower + POWER_STEP, POWER_STEP, 1);
            if (gamepad1.dpadDownWasPressed()) outtakePower = Range.clip(outtakePower - POWER_STEP, POWER_STEP, 1);
            outtake.setPower(outtakeOn ? outtakePower : 0);

            double ticksPerSec = Math.abs(outtake.getVelocity());
            maxTicksPerSec = Math.max(maxTicksPerSec, ticksPerSec);
            if (gamepad1.leftBumperWasPressed()) maxTicksPerSec = 0;

            if (gate != null && gamepad1.circleWasPressed()) {
                gateOpen = !gateOpen;
                gate.setPosition(gateOpen ? BallPath.GATE_OPEN : BallPath.GATE_CLOSED);
                gateState = gateOpen ? "OPEN" : "SHUT";
            }

            telemetry.addData("Intake", intakeState);
            telemetry.addData("Outtake", "%s at %.0f%% power", outtakeOn ? "ON" : "off", 100 * outtakePower);
            telemetry.addData("Outtake speed", "%.0f ticks/s  (highest %.0f)", ticksPerSec, maxTicksPerSec);
            if (outtakeOn && ticksPerSec == 0 && outtakeOnTimer.seconds() > ENCODER_CHECK_S) {
                telemetry.addLine("Speed reads 0: encoder cable not in Expansion Hub encoder port 0");
            }
            telemetry.addData("Gate", gateState);
            telemetry.addLine("Wrong way? Intake must pull balls IN; outtake must throw them OUT.");
            telemetry.update();
        }

        intake.setPower(0);
        outtake.setPower(0);
    }
}
