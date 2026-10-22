package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

// Just the four drive wheels: drive, strafe and turn. Uses the Control Hub's
// motor ports directly, so the robot config doesn't matter.
//   Control Hub port 1 = front left, 0 = front right, 3 = back left,
//   2 = back right (same wiring as meet 1, so the same directions as
//   hardware/MecanumDrive: left side reversed).
//   Left stick      drive / strafe (up = toward the intake)
//   Right stick X   turn
//   L2 hold         slow mode
@TeleOp(name = "Drive Only TeleOp", group = "teleOp")
public class DriveOnlyTeleOp extends LinearOpMode {

    private static final int FRONT_LEFT_PORT = 1;
    private static final int FRONT_RIGHT_PORT = 0;
    private static final int BACK_LEFT_PORT = 3;
    private static final int BACK_RIGHT_PORT = 2;

    private static final double SLOW_SCALE = 0.35;
    private static final double TRIGGER_PRESSED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule controlHub = null;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent()) controlHub = module;
        }
        if (controlHub == null) {
            telemetry.addLine("Control Hub not found.");
            telemetry.update();
            waitForStart();
            return;
        }
        LynxDcMotorController ports;
        try {
            ports = new LynxDcMotorController(hardwareMap.appContext, controlHub);
        } catch (RobotCoreException e) {
            telemetry.addData("Couldn't open the Control Hub's motor ports", e.toString());
            telemetry.update();
            waitForStart();
            return;
        }
        DcMotor frontLeft = new DcMotorImplEx(ports, FRONT_LEFT_PORT, DcMotor.Direction.REVERSE);
        DcMotor frontRight = new DcMotorImplEx(ports, FRONT_RIGHT_PORT, DcMotor.Direction.FORWARD);
        DcMotor backLeft = new DcMotorImplEx(ports, BACK_LEFT_PORT, DcMotor.Direction.REVERSE);
        DcMotor backRight = new DcMotorImplEx(ports, BACK_RIGHT_PORT, DcMotor.Direction.FORWARD);
        DcMotor[] wheels = {frontLeft, frontRight, backLeft, backRight};
        for (DcMotor wheel : wheels) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Ready. Left stick = drive/strafe, right stick = turn, L2 = slow.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                double scale = gamepad1.left_trigger > TRIGGER_PRESSED ? SLOW_SCALE : 1.0;
                double forward = -gamepad1.left_stick_y * scale;
                double strafeRight = gamepad1.left_stick_x * scale;
                double turnClockwise = gamepad1.right_stick_x * scale;

                double fl = forward + strafeRight + turnClockwise;
                double fr = forward - strafeRight - turnClockwise;
                double bl = forward - strafeRight + turnClockwise;
                double br = forward + strafeRight - turnClockwise;
                // Scale all four down together if any would go past full power.
                double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))));

                frontLeft.setPower(fl / max);
                frontRight.setPower(fr / max);
                backLeft.setPower(bl / max);
                backRight.setPower(br / max);

                telemetry.addData("Front L / R", "%+.2f  %+.2f", fl / max, fr / max);
                telemetry.addData("Back  L / R", "%+.2f  %+.2f", bl / max, br / max);
                telemetry.addData("Slow mode (L2)", scale < 1 ? "ON" : "off");
                telemetry.update();
            }
        } finally {
            for (DcMotor wheel : wheels) wheel.setPower(0);
        }
    }
}
