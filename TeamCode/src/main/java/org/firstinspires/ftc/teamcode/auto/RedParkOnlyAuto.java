package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Parking robot, red: see ParkOnlyAutoBase.
@Autonomous(name = "Red Auto - Park Only", group = "Autonomous")
public class RedParkOnlyAuto extends ParkOnlyAutoBase {
    public RedParkOnlyAuto() {
        alliance = FieldConstants.Alliance.RED;
    }
}
