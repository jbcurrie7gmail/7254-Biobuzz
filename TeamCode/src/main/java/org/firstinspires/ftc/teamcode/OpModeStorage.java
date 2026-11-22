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

    static void beginAutonomous(Alliance selectedAlliance) {
        alliance = selectedAlliance;
        autonomousEndPose = null;
    }
}
