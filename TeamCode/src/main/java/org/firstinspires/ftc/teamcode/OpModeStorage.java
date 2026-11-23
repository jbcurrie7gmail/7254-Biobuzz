package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class OpModeStorage {
    enum Alliance {
        BLUE(1),
        RED(0);

        final int aprilTagPipeline;

        Alliance(int aprilTagPipeline) {
            this.aprilTagPipeline = aprilTagPipeline;
        }
    }

    static Alliance alliance = Alliance.BLUE;
    public static Pose autonomousEndPose;
    static HiveUpRouteChecks.FlowerExit autonomousIntakeExit;
    static double autonomousIntakeDownPosition;

    static void beginAutonomous(Alliance selectedAlliance) {
        alliance = selectedAlliance;
        saveAutonomousEnd(null, null, 0);
    }

    static void saveAutonomousEnd(Pose pose, HiveUpRouteChecks.FlowerExit intakeExit,
                                  double intakeDownPosition) {
        autonomousEndPose = pose;
        autonomousIntakeExit = intakeExit;
        autonomousIntakeDownPosition = intakeDownPosition;
    }
}
