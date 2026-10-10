package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Meet 1, for when the PARTNER shoots the preloads (two shooters at one CELL
// waste balls): see MeetOneAutoBase. Start with the left side on the blue
// alliance wall, just short of the LOADING ZONE. Keeps its 4 POLLEN.
@Autonomous(name = "BLUE Leave+Park (no shoot)", group = "Meet 1", preselectTeleOp = "Meet 1 TeleOp")
public class MeetOneBluePark extends MeetOneAutoBase {
    public MeetOneBluePark() {
        alliance = FieldConstants.Alliance.BLUE;
        shootPreload = false;
    }
}
