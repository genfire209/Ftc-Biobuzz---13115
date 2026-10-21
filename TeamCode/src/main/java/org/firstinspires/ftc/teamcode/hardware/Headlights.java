package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// The two goBILDA LED Headlights on the sides of the Nemesis ball path.
// They plug into servo ports and the servo signal sets the brightness
// (position 0 = off, 1 = full).
//   - "headlight_left"  (Expansion Hub servo port 1)
//   - "headlight_right" (Expansion Hub servo port 2)
// Optional: missing ones are skipped.
public class Headlights {

    private static final String[] NAMES = {"headlight_left", "headlight_right"};

    private final Servo[] lights = new Servo[NAMES.length];

    public void init(HardwareMap hw) {
        for (int i = 0; i < NAMES.length; i++) lights[i] = hw.tryGet(Servo.class, NAMES[i]);
    }

    // brightness: 0 = off, 1 = full.
    public void set(double brightness) {
        for (Servo light : lights) {
            if (light != null) light.setPosition(brightness);
        }
    }
}
