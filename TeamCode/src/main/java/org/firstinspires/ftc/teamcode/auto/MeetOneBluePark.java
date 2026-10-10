package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Meet 1: see MeetOneAutoBase. Left side on the blue alliance wall, just short of the LOADING ZONE.
@Autonomous(name = "BLUE Park Only", group = "Meet 1", preselectTeleOp = "Meet 1 TeleOp")
public class MeetOneBluePark extends MeetOneAutoBase {
    public MeetOneBluePark() {
        alliance = FieldConstants.Alliance.BLUE;
        shootPreload = false;
    }
}
