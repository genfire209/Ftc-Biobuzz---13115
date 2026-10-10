package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;
import java.util.Locale;

// One number kept in a file on the Control Hub (FIRST settings folder), so a
// value tuned on the robot survives restarts and app installs without a
// rebuild. Used for the gate positions, the TEMP TeleOp outtake power and
// the timed auto's drive time.
public final class SavedNumber {

    private SavedNumber() {}

    // Returns `fallback` if nothing has been saved yet or the file is bad.
    public static double load(String name, double fallback) {
        File file = AppUtil.getInstance().getSettingsFile(name + ".txt");
        if (!file.exists()) return fallback;
        try {
            return Double.parseDouble(ReadWriteFile.readFileOrThrow(file).trim());
        } catch (IOException | NumberFormatException e) {
            return fallback;
        }
    }

    public static boolean save(String name, double value) {
        File file = AppUtil.getInstance().getSettingsFile(name + ".txt");
        try {
            ReadWriteFile.writeFileOrThrow(file, String.format(Locale.US, "%.3f\n", value));
            return true;
        } catch (IOException e) {
            return false;
        }
    }
}
