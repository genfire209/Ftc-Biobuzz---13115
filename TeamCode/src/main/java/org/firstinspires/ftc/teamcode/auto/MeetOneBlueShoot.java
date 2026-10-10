package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Meet 1: see MeetOneAutoBase. Back on the far wall, centered on the blue HIVE, facing it.
@Autonomous(name = "BLUE Shoot+Park", group = "Meet 1", preselectTeleOp = "Meet 1 TeleOp")
public class MeetOneBlueShoot extends MeetOneAutoBase {
    public MeetOneBlueShoot() {
        alliance = FieldConstants.Alliance.BLUE;
        shootPreload = true;
    }
}
