package org.firstinspires.ftc.teamcode.util.field;

public class AllianceColor {
    public enum Selection {
        RED,
        BLUE
    }

    Selection allianceColor;
    public AllianceColor(Selection allianceColor) {
        this.allianceColor = allianceColor;
    }

    public boolean isRed() {
        return allianceColor == Selection.RED;
    }

    public Selection getSelection() {
        return allianceColor;
    }
}
