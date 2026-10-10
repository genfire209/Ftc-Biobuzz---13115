package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Meet 1: see MeetOneAutoBase. Back on the audience wall, centered on the red HIVE, facing it.
@Autonomous(name = "RED Shoot+Park", group = "Meet 1", preselectTeleOp = "Meet 1 TeleOp")
public class MeetOneRedShoot extends MeetOneAutoBase {
    public MeetOneRedShoot() {
        alliance = FieldConstants.Alliance.RED;
        shootPreload = true;
    }
}
