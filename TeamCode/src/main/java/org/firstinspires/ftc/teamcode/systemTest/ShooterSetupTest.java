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
import org.firstinspires.ftc.teamcode.hardware.SavedNumber;

// Finds the numbers the meet-1 shooter code needs. The gate numbers are
// saved on the hub as soon as you set them (every OpMode uses them from the
// next INIT, no rebuild); MAX_TICKS_PER_SEC goes into hardware/Flywheel.java.
//
//   Right stick up       flywheel power (raw). Telemetry shows ticks/s and
//                        the highest seen -> Flywheel.MAX_TICKS_PER_SEC
//                        (full stick, charged battery, no ball).
//                        "ticks/s 0" at full power = encoder not plugged in.
//   L1                   reset the highest-seen reading
//   R2 / L2 hold         intake in / out (load balls up to the gate)
//   D-pad up / down      move the gate servo +/-0.02
//   Square / Triangle    save this position as SHUT / OPEN (on the hub)
//   D-pad left / right   gate pulse length -/+0.05 s (saved on the hub)
//   Cross                one pulse: OPEN for the pulse length, then SHUT.
//                        Right = exactly one ball gets through per pulse.
@TeleOp(name = "Shooter Setup Test", group = "systemTest")
public class ShooterSetupTest extends LinearOpMode {

    private static final double GATE_STEP = 0.02;
    private static final double PULSE_STEP_S = 0.05;
    private static final double TRIGGER_PRESSED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, Flywheel.MOTOR_NAME);
        flywheel.setDirection(Flywheel.DIRECTION);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotor intake = hardwareMap.get(DcMotor.class, BallPath.INTAKE_NAME);
        intake.setDirection(BallPath.INTAKE_DIRECTION);
        Servo gate = hardwareMap.tryGet(Servo.class, "shooter_gate");

        double shut = BallPath.savedGateClosed();
        double open = BallPath.savedGateOpen();
        double pulseS = BallPath.savedOpenPulseS();
        double gatePosition = shut;
        String saveNote = "";
        double maxTicksPerSec = 0;
        boolean pulsing = false;
        ElapsedTime pulseTimer = new ElapsedTime();

        if (gate != null) gate.setPosition(gatePosition);
        telemetry.addLine(gate == null ? "No 'shooter_gate' servo in the config!" : "Ready.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double power = Range.clip(-gamepad1.right_stick_y, 0, 1);
            flywheel.setPower(power);
            double ticksPerSec = Math.abs(flywheel.getVelocity());
            maxTicksPerSec = Math.max(maxTicksPerSec, ticksPerSec);
            if (gamepad1.leftBumperWasPressed()) maxTicksPerSec = 0;

            if (gamepad1.right_trigger > TRIGGER_PRESSED) {
                intake.setPower(BallPath.INTAKE_POWER);
            } else if (gamepad1.left_trigger > TRIGGER_PRESSED) {
                intake.setPower(BallPath.SPIT_POWER);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.dpadUpWasPressed()) gatePosition = Range.clip(gatePosition + GATE_STEP, 0, 1);
            if (gamepad1.dpadDownWasPressed()) gatePosition = Range.clip(gatePosition - GATE_STEP, 0, 1);
            if (gamepad1.squareWasPressed()) {
                shut = gatePosition;
                saveNote = SavedNumber.save(BallPath.GATE_CLOSED_FILE, shut) ? "SHUT saved" : "SAVE FAILED";
            }
            if (gamepad1.triangleWasPressed()) {
                open = gatePosition;
                saveNote = SavedNumber.save(BallPath.GATE_OPEN_FILE, open) ? "OPEN saved" : "SAVE FAILED";
            }
            boolean pulseShorter = gamepad1.dpadLeftWasPressed();
            boolean pulseLonger = gamepad1.dpadRightWasPressed();
            if (pulseShorter) pulseS = Math.max(PULSE_STEP_S, pulseS - PULSE_STEP_S);
            if (pulseLonger) pulseS += PULSE_STEP_S;
            if (pulseShorter || pulseLonger) {
                saveNote = SavedNumber.save(BallPath.OPEN_PULSE_FILE, pulseS) ? "pulse saved" : "SAVE FAILED";
            }
            if (gamepad1.crossWasPressed() && !pulsing) {
                pulsing = true;
                pulseTimer.reset();
            }

            double commanded = gatePosition;
            if (pulsing) {
                commanded = pulseTimer.seconds() < pulseS ? open : shut;
                if (pulseTimer.seconds() >= pulseS) {
                    pulsing = false;
                    gatePosition = shut;
                }
            }
            if (gate != null) gate.setPosition(commanded);

            telemetry.addData("Flywheel power", "%.2f", power);
            telemetry.addData("Flywheel ticks/s", "%.0f   (highest %.0f -> MAX_TICKS_PER_SEC)", ticksPerSec, maxTicksPerSec);
            telemetry.addData("Gate position now", "%.2f", commanded);
            telemetry.addData("SHUT / OPEN (saved)", "%.2f / %.2f", shut, open);
            telemetry.addData("Pulse length (saved)", "%.2f s", pulseS);
            telemetry.addData("Last save", saveNote);
            telemetry.update();
        }

        flywheel.setPower(0);
        intake.setPower(0);
    }
}
