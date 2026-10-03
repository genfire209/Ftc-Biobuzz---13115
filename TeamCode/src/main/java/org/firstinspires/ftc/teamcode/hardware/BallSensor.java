package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// REV Color Sensor V3 at the last spot before the flywheel, where the next
// ball to be fired waits. Tells POLLEN (yellow) from NECTAR (red/blue) so
// the launcher can pick the right RPM table.
//
// OPTIONAL: if no device named SENSOR_NAME is in the robot configuration
// (it isn't bought yet), isPresent() is false and read() reports UNKNOWN,
// which the launcher treats as POLLEN -- the auto still runs.
//
// All thresholds are starting guesses -- TODO: TUNE with
// systemTest/BallSensorTest (hold each ball in the slot, read the hue and
// distance it shows, then tighten these ranges).
public class BallSensor {

    public enum BallType { EMPTY, POLLEN, NECTAR, UNKNOWN }

    // TODO: DECIDE -- must match the Driver Hub config name (I2C device type
    // "REV Color Sensor V3").
    private static final String SENSOR_NAME = "ball_sensor";

    // TODO: TUNE -- sensor gain; raise it if readings come back dark.
    private static final float GAIN = 2.0f;

    // TODO: TUNE -- a ball is in the slot when it's closer than this.
    private static final double BALL_PRESENT_CM = 3.0;

    // TODO: TUNE -- hue ranges in degrees (0-360).
    private static final float POLLEN_HUE_MIN = 35f, POLLEN_HUE_MAX = 80f;   // yellow
    private static final float RED_HUE_BELOW = 25f, RED_HUE_ABOVE = 330f;    // red wraps around 0
    private static final float BLUE_HUE_MIN = 190f, BLUE_HUE_MAX = 250f;

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private final float[] hsv = new float[3];

    public void init(HardwareMap hw) {
        colorSensor = hw.tryGet(NormalizedColorSensor.class, SENSOR_NAME);
        if (colorSensor != null) {
            colorSensor.setGain(GAIN);
            distanceSensor = (colorSensor instanceof DistanceSensor) ? (DistanceSensor) colorSensor : null;
        }
    }

    public boolean isPresent() {
        return colorSensor != null;
    }

    public BallType read() {
        if (colorSensor == null) return BallType.UNKNOWN;
        if (distanceSensor != null && getDistanceCm() > BALL_PRESENT_CM) return BallType.EMPTY;

        float hue = getHue();
        if (hue >= POLLEN_HUE_MIN && hue <= POLLEN_HUE_MAX) return BallType.POLLEN;
        if (hue <= RED_HUE_BELOW || hue >= RED_HUE_ABOVE) return BallType.NECTAR;
        if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) return BallType.NECTAR;
        return BallType.UNKNOWN;
    }

    // Raw values for the bench test / telemetry.
    public float getHue() {
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }

    public double getDistanceCm() {
        return (distanceSensor == null) ? Double.NaN : distanceSensor.getDistance(DistanceUnit.CM);
    }
}
