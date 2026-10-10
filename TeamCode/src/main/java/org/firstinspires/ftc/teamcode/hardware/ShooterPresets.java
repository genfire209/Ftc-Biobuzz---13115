package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;
import java.util.Locale;

// Flywheel speeds (fraction of full speed, see Flywheel) for the two
// shooting spots used at meet 1:
//   WALL     - robot's back against the audience or far wall, centered on
//              our HIVE, facing it. The far CELL from the far wall mirrors
//              the audience CELL from the audience wall, so one number
//              covers both. The auto shoots from here too.
//   ONE_TILE - backup when a robot is in the way: one tile (23.5in) off
//              the wall, same line.
//
// Tuned live in MeetOneTeleOp (D-pad left/right) and saved on the Control
// Hub with Share, so practice-field tuning carries into matches without a
// rebuild. Auto and TeleOp load the file at INIT; the defaults below are
// only used until the first save.
public class ShooterPresets {

    public enum Spot { WALL, ONE_TILE }

    // TODO: TUNE at the practice HIVE (then press Share to save).
    private static final double DEFAULT_WALL = 0.70;
    private static final double DEFAULT_ONE_TILE = 0.75;

    private static final double MIN = 0.20;
    private static final double MAX = 1.00;

    private static final String FILE_NAME = "meet1_shooter_presets.txt";

    private double wall = DEFAULT_WALL;
    private double oneTile = DEFAULT_ONE_TILE;
    private boolean loadedFromFile = false;

    // Keeps the defaults if the file is missing or unreadable.
    public void load() {
        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
        if (!file.exists()) return;
        try {
            for (String line : ReadWriteFile.readFileOrThrow(file).split("\n")) {
                String[] kv = line.trim().split("=");
                if (kv.length != 2) continue;
                double value = Range.clip(Double.parseDouble(kv[1].trim()), MIN, MAX);
                if (kv[0].trim().equals("wall")) wall = value;
                if (kv[0].trim().equals("oneTile")) oneTile = value;
            }
            loadedFromFile = true;
        } catch (IOException | NumberFormatException e) {
            wall = DEFAULT_WALL;
            oneTile = DEFAULT_ONE_TILE;
        }
    }

    public boolean save() {
        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
        try {
            ReadWriteFile.writeFileOrThrow(file,
                    String.format(Locale.US, "wall=%.3f\noneTile=%.3f\n", wall, oneTile));
            loadedFromFile = true;
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    public double get(Spot spot) {
        return spot == Spot.WALL ? wall : oneTile;
    }

    public void trim(Spot spot, double delta) {
        if (spot == Spot.WALL) {
            wall = Range.clip(wall + delta, MIN, MAX);
        } else {
            oneTile = Range.clip(oneTile + delta, MIN, MAX);
        }
    }

    public boolean isLoadedFromFile() { return loadedFromFile; }
}
