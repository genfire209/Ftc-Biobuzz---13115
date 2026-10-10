package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Meet 1: see MeetOneAutoBase. Left side on the red alliance wall, just short of the LOADING ZONE.
@Autonomous(name = "RED Park Only", group = "Meet 1", preselectTeleOp = "Meet 1 TeleOp")
public class MeetOneRedPark extends MeetOneAutoBase {
    public MeetOneRedPark() {
        alliance = FieldConstants.Alliance.RED;
        shootPreload = false;
    }
}
