package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

final class HiveUpRouteChecks {
    private HiveUpRouteChecks() {
    }

    static boolean near(Pose pose, Pose target, double tolerance) {
        return Math.hypot(pose.x() - target.x(), pose.y() - target.y()) <= tolerance;
    }

    static boolean atShot(Pose pose, Pose target, double positionTolerance, double headingTolerance) {
        double error = target.heading() - pose.heading();
        return near(pose, target, positionTolerance)
                && Math.abs(Math.atan2(Math.sin(error), Math.cos(error))) <= headingTolerance;
    }

    static final class FlowerExit {
        private final Pose boundary;
        private final double directionX;
        private final double directionY;
        private final double lateralTolerance;
        final Pose target;

        FlowerExit(Pose inside, Pose boundary, double margin) {
            this(inside, boundary, margin, Double.POSITIVE_INFINITY);
        }

        FlowerExit(Pose inside, Pose boundary, double margin, double lateralTolerance) {
            this.boundary = boundary;
            this.lateralTolerance = lateralTolerance;
            double heading = Math.atan2(boundary.y() - inside.y(), boundary.x() - inside.x());
            directionX = Math.cos(heading);
            directionY = Math.sin(heading);
            target = new Pose(boundary.x() + margin * directionX,
                    boundary.y() + margin * directionY, boundary.heading());
        }

        boolean isClear(Pose pose) {
            double x = pose.x() - boundary.x();
            double y = pose.y() - boundary.y();
            return x * directionX + y * directionY >= 0
                    && Math.abs(x * directionY - y * directionX) <= lateralTolerance;
        }
    }
}
