package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Shooting robot, red. Start touching the audience wall in column C,
// straight in front of the red HIVE's upward CELL, facing it. Poses live
// in FieldConstants.
@Autonomous(name = "Red Auto - Two Tips", group = "Autonomous")
public class RedAuto extends HiveShootAutoBase {
    public RedAuto() {
        alliance = FieldConstants.Alliance.RED;
    }
}
