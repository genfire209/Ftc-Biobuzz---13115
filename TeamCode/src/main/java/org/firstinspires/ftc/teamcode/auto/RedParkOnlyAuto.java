package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Parking robot, red: see ParkOnlyAutoBase.
// Hidden for meet 1: relies on an untuned Pedro follower and an 18in-robot
// path. Use the "Meet 1" autos instead.
@Disabled
@Autonomous(name = "Red Auto - Park Only", group = "Autonomous")
public class RedParkOnlyAuto extends ParkOnlyAutoBase {
    public RedParkOnlyAuto() {
        alliance = FieldConstants.Alliance.RED;
    }
}
