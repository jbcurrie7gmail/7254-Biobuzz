package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.robotcore.external.Telemetry;

final class AutonParking {
    private boolean parkOne;

    void selectParkOne(boolean selected) {
        parkOne = selected;
    }

    void show(Telemetry telemetry, Vector2D first, Vector2D second) {
        Vector2D spot = parkOne ? first : second;
        telemetry.addData("Park target", "Park %d (%.1f, %.1f)", parkOne ? 1 : 2, spot.x(), spot.y());
    }

    Pose targetFrom(Pose current, Vector2D first, Vector2D second) {
        Vector2D spot = parkOne ? first : second;
        double heading = current.y() > spot.y() ? 90 : 270;
        return new Pose(spot.x(), spot.y(), Math.toRadians(heading));
    }
}
