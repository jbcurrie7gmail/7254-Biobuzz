package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

final class AutonInitMenu {
    private final Row shooting = new Row("Shooting mode", 0, "Shoot 1 side", "Shoot 2 sides");
    private final Row alliance;
    private final Row parking = new Row("Parking", 1, "Park 1", "Park 2");
    private final Row[] rows;
    private final OpModeStorage.Alliance[] alliances;
    private int cursor;

    AutonInitMenu(boolean hiveUp) {
        if (hiveUp) {
            alliance = new Row("Alliance", 0, "Auto", "Blue", "Red");
            alliances = new OpModeStorage.Alliance[]{null, OpModeStorage.Alliance.BLUE,
                    OpModeStorage.Alliance.RED};
            rows = new Row[]{shooting, alliance, parking};
        } else {
            alliance = new Row("Alliance", 0, "Blue", "Red");
            alliances = new OpModeStorage.Alliance[]{OpModeStorage.Alliance.BLUE,
                    OpModeStorage.Alliance.RED};
            rows = new Row[]{alliance, parking};
        }
    }

    void update(Gamepad gamepad) {
        if (gamepad.dpadUpWasPressed()) {
            cursor = Math.floorMod(cursor - 1, rows.length);
        } else if (gamepad.dpadDownWasPressed()) {
            cursor = (cursor + 1) % rows.length;
        }

        Row row = rows[cursor];
        if (gamepad.dpadLeftWasPressed()) {
            row.preview = Math.floorMod(row.preview - 1, row.choices.length);
        } else if (gamepad.dpadRightWasPressed()) {
            row.preview = (row.preview + 1) % row.choices.length;
        }
        if (gamepad.xWasPressed()) {
            row.confirmed = row.preview;
        }
    }

    void show(Telemetry telemetry) {
        telemetry.addData("Gamepad 2", "D-pad up/down: row | left/right: choice | X: confirm");
        for (int index = 0; index < rows.length; index++) {
            Row row = rows[index];
            String value = row.choices[row.confirmed];
            if (row.preview != row.confirmed) {
                value += " | Preview: " + row.choices[row.preview];
            }
            telemetry.addData((index == cursor ? "> " : "  ") + row.label, value);
        }
    }

    boolean shootTwoSides() {
        return shooting.confirmed == 1;
    }

    OpModeStorage.Alliance alliance() {
        return alliances[alliance.confirmed];
    }

    boolean parkOne() {
        return parking.confirmed == 0;
    }

    private static final class Row {
        final String label;
        final String[] choices;
        int confirmed;
        int preview;

        Row(String label, int initial, String... choices) {
            this.label = label;
            this.choices = choices;
            confirmed = initial;
            preview = initial;
        }
    }
}
