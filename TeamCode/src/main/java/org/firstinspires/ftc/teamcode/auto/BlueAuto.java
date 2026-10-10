package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Shooting robot, blue. Start touching the far (back) wall in column D,
// straight in front of the blue HIVE's upward CELL, facing it. Poses are
// the red ones rotated 180 deg (FieldConstants.forAlliance).
// Hidden for meet 1: needs a turret, feeder motor and tuned Pedro that this
// robot doesn't have (crashes at INIT). Use the "Meet 1" autos instead.
@Disabled
@Autonomous(name = "Blue Auto - Two Tips", group = "Autonomous")
public class BlueAuto extends HiveShootAutoBase {
    public BlueAuto() {
        alliance = FieldConstants.Alliance.BLUE;
    }
}
