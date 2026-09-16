package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// TODO: MEASURE -- startPose/firingPose/parkPose are placeholders until the
// team picks an AUTO starting tile. Add more subclasses (e.g. BlueAutoLeft/
// BlueAutoRight) the same way if more than one starting position is needed.
@Autonomous(name = "Blue Auto", group = "Autonomous")
public class BlueAuto extends HiveShootAutoBase {
    public BlueAuto() {
        startPose = new Pose(144, 0, Math.toRadians(180));
        firingPose = new Pose(120, 24, Math.toRadians(180));
        parkPose = new Pose(132, 12, Math.toRadians(180));
        hiveTarget = FieldConstants.BLUE_HIVE_TARGET;
    }
}
