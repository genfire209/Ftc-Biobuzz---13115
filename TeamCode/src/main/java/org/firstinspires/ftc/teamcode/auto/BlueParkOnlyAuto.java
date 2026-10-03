package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Parking robot, blue: see ParkOnlyAutoBase.
@Autonomous(name = "Blue Auto - Park Only", group = "Autonomous")
public class BlueParkOnlyAuto extends ParkOnlyAutoBase {
    public BlueParkOnlyAuto() {
        alliance = FieldConstants.Alliance.BLUE;
    }
}
