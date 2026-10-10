package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SavedNumber;

// TIMED LEAVE + PARK, for a robot with no odometry pods: it drives on
// timers only, so the same program works on either alliance.
//
//   Start: robot's LEFT side touching our alliance wall (the wall the drive
//   team stands behind), intake facing our LOADING ZONE, front just short
//   of the zone's tape (starting inside it is illegal, G304E).
//   Then it:
//     1. waits the start delay,
//     2. strafes right off the wall (LEAVE = not touching the wall, 3),
//     3. drives forward into the LOADING ZONE (PARK, 5) and stops.
//   No shooting: it keeps its POLLEN for TELEOP.
//
// INIT menu (gamepad 1): D-pad up/down = start delay, D-pad left/right =
// forward drive time (saved on the hub). Stops short of the zone -> more
// time; goes too far -> less.
//
// Drive power is scaled by battery voltage so the distance stays about the
// same on a fresh or a tired battery.
@Autonomous(name = "TIMED Leave+Park (no pods)", group = "Meet 1", preselectTeleOp = "TEMP TeleOp (simple)")
public class TimedLeaveParkAuto extends LinearOpMode {

    private static final double DRIVE_POWER = 0.30;
    private static final double NOMINAL_VOLTAGE = 12.5;

    // Guesses (nothing to measure with yet): ~6in off the wall, then ~14in
    // forward -- front about halfway into the zone. Anything from about half
    // to double that distance still ends partly in the zone.
    private static final double STRAFE_S = 0.5;
    private static final double DEFAULT_FORWARD_S = 0.8;
    private static final double FORWARD_STEP_S = 0.1;
    private static final double MIN_FORWARD_S = 0.2;
    private static final double MAX_FORWARD_S = 3.0;
    private static final String FORWARD_FILE = "meet1_timed_park_forward_s";

    // Pause between the strafe and the forward drive so the robot settles.
    private static final double SETTLE_S = 0.3;
    private static final double MAX_DELAY_S = 20.0;

    private final MecanumDrive drive = new MecanumDrive();
    private String step = "waiting for START";

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;

        double forwardS = Range.clip(SavedNumber.load(FORWARD_FILE, DEFAULT_FORWARD_S), MIN_FORWARD_S, MAX_FORWARD_S);
        double delayS = 0;
        String saveNote = "saved on hub";

        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) delayS = Math.min(MAX_DELAY_S, delayS + 1);
            if (gamepad1.dpadDownWasPressed()) delayS = Math.max(0, delayS - 1);
            double before = forwardS;
            if (gamepad1.dpadRightWasPressed()) forwardS += FORWARD_STEP_S;
            if (gamepad1.dpadLeftWasPressed()) forwardS -= FORWARD_STEP_S;
            forwardS = Range.clip(forwardS, MIN_FORWARD_S, MAX_FORWARD_S);
            if (forwardS != before) {
                saveNote = SavedNumber.save(FORWARD_FILE, forwardS) ? "saved on hub" : "SAVE FAILED";
            }

            telemetry.addLine("LEFT side on our alliance wall, intake facing our LOADING ZONE,");
            telemetry.addLine("front just short of the zone tape.");
            telemetry.addData("Start delay (D-pad up/down)", "%.0f s", delayS);
            telemetry.addData("Forward time (D-pad left/right)", "%.1f s  (%s)", forwardS, saveNote);
            if (battery != null) telemetry.addData("Battery", "%.1f V", battery.getVoltage());
            telemetry.update();
        }
        if (isStopRequested()) return;

        // Battery read once: it sags while driving, and these moves are short.
        double voltage = battery != null ? battery.getVoltage() : NOMINAL_VOLTAGE;
        double power = Range.clip(DRIVE_POWER * NOMINAL_VOLTAGE / Math.max(voltage, 6.0), 0, 1);

        step = "start delay";
        runFor(delayS, 0, 0);
        step = "strafe off the wall (LEAVE)";
        runFor(STRAFE_S, 0, power);
        step = "settle";
        runFor(SETTLE_S, 0, 0);
        step = "forward into the LOADING ZONE (PARK)";
        runFor(forwardS, power, 0);
        drive.stop();

        step = "done";
        while (opModeIsActive()) {
            telemetry.addData("Step", step);
            telemetry.update();
        }
    }

    // Drives robot-centric for `seconds` (0 power = stand still), then stops.
    private void runFor(double seconds, double forward, double strafeRight) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drive.drive(forward, strafeRight, 0);
            telemetry.addData("Step", step);
            telemetry.addData("Time", "%.1f of %.1f s", timer.seconds(), seconds);
            telemetry.update();
        }
        drive.stop();
    }
}
