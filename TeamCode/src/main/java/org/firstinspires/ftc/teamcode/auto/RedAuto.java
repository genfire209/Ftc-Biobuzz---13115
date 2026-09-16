package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// TODO: MEASURE -- startPose/firingPose/parkPose are placeholders until the
// team picks an AUTO starting tile. Add more subclasses (e.g. RedAutoLeft/
// RedAutoRight) the same way if more than one starting position is needed.
@Autonomous(name = "Red Auto", group = "Autonomous")
public class RedAuto extends HiveShootAutoBase {
    public RedAuto() {
        startPose = new Pose(0, 0, 0);
        firingPose = new Pose(24, 24, 0);
        parkPose = new Pose(12, 12, 0);
        hiveTarget = FieldConstants.RED_HIVE_TARGET;
    }
}
