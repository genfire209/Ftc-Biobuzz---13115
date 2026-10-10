package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Parking robot, blue: see ParkOnlyAutoBase.
// Hidden for meet 1: relies on an untuned Pedro follower and an 18in-robot
// path. Use the "Meet 1" autos instead.
@Disabled
@Autonomous(name = "Blue Auto - Park Only", group = "Autonomous")
public class BlueParkOnlyAuto extends ParkOnlyAutoBase {
    public BlueParkOnlyAuto() {
        alliance = FieldConstants.Alliance.BLUE;
    }
}
