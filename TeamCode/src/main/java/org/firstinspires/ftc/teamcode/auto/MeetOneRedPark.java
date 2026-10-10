package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.FieldConstants;

// Meet 1, for when the PARTNER shoots the preloads (two shooters at one CELL
// waste balls): see MeetOneAutoBase. Start with the left side on the red
// alliance wall, just short of the LOADING ZONE on its middle-of-the-field
// side. Keeps its 4 POLLEN.
@Autonomous(name = "RED Leave+Park (no shoot)", group = "Meet 1", preselectTeleOp = "Meet 1 TeleOp")
public class MeetOneRedPark extends MeetOneAutoBase {
    public MeetOneRedPark() {
        alliance = FieldConstants.Alliance.RED;
        shootPreload = false;
    }
}
