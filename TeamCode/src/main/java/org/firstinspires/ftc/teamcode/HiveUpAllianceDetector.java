package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

final class HiveUpAllianceDetector {
    static final long MAX_RESULT_AGE_MS = 200;
    private static final long PIPELINE_SCAN_NS = 700_000_000L;

    private final Limelight3A limelight;
    private OpModeStorage.Alliance scanning;
    private OpModeStorage.Alliance detected;
    private OpModeStorage.Alliance override;
    private long switchTimeNs;
    private long frameBeforeSwitch;

    HiveUpAllianceDetector(Limelight3A limelight) {
        this.limelight = limelight;
        switchTo(OpModeStorage.Alliance.BLUE);
    }

    void update(OpModeStorage.Alliance selectedOverride, Telemetry telemetry) {
        setOverride(selectedOverride);
        if (override != null) {
            telemetry.addData("Alliance status", "%s (manual), pipeline %d",
                    override, override.aprilTagPipeline);
            return;
        }

        LLResult frame = limelight.getLatestResult();
        if (isFreshForPipeline(frame)) {
            List<LLResultTypes.FiducialResult> tags = frame.getFiducialResults();
            if (tags != null) {
                for (LLResultTypes.FiducialResult tag : tags) {
                    if (isAllianceTag(tag.getFiducialId())) {
                        detected = scanning;
                        addTagPose(telemetry, tag);
                    }
                }
            }
        }

        if (detected == null && System.nanoTime() - switchTimeNs >= PIPELINE_SCAN_NS) {
            switchTo(scanning == OpModeStorage.Alliance.BLUE
                    ? OpModeStorage.Alliance.RED : OpModeStorage.Alliance.BLUE);
        }

        if (detected == null) {
            telemetry.addData("Alliance status", "Auto: scanning %s pipeline %d",
                    scanning, scanning.aprilTagPipeline);
            telemetry.addData("Fallback at START", OpModeStorage.Alliance.BLUE);
        } else {
            telemetry.addData("Alliance status", "Auto: %s locked on pipeline %d",
                    detected, detected.aprilTagPipeline);
        }
    }

    OpModeStorage.Alliance finish() {
        OpModeStorage.Alliance selected = selectedAlliance();
        if (scanning != selected) {
            switchTo(selected);
        }
        return selected;
    }

    OpModeStorage.Alliance selectedAlliance() {
        if (override != null) {
            return override;
        }
        return detected == null ? OpModeStorage.Alliance.BLUE : detected;
    }

    String selectionSource() {
        if (override != null) {
            return "manual";
        }
        return detected == null ? "fallback" : "detected";
    }

    private void setOverride(OpModeStorage.Alliance selected) {
        if (override == selected) {
            return;
        }
        override = selected;
        if (selected == null) {
            detected = null;
            switchTo(OpModeStorage.Alliance.BLUE);
        } else if (scanning != selected) {
            switchTo(selected);
        }
    }

    private void switchTo(OpModeStorage.Alliance alliance) {
        LLResult latest = limelight.getLatestResult();
        frameBeforeSwitch = latest == null ? -1 : latest.getControlHubTimeStamp();
        scanning = alliance;
        limelight.pipelineSwitch(alliance.aprilTagPipeline);
        switchTimeNs = System.nanoTime();
    }

    private boolean isFreshForPipeline(LLResult frame) {
        return frame != null && frame.isValid()
                && frame.getPipelineIndex() == scanning.aprilTagPipeline
                && frame.getStaleness() <= MAX_RESULT_AGE_MS
                && frame.getControlHubTimeStamp() > frameBeforeSwitch;
    }

    private boolean isAllianceTag(int id) {
        if (scanning == OpModeStorage.Alliance.RED) {
            return id == 31 || id == 32 || id == 35 || id == 36;
        }
        return id == 39 || id == 40 || id == 43 || id == 44;
    }

    private void addTagPose(Telemetry telemetry, LLResultTypes.FiducialResult tag) {
        Pose3D pose = tag.getRobotPoseTargetSpace();
        if (pose != null) {
            Position offset = pose.getPosition().toUnit(DistanceUnit.INCH);
            telemetry.addData("Robot from tag " + tag.getFiducialId() + " (in)",
                    "X (sideways) %+.2f, Y (out from tag) %+.2f", offset.x, offset.z);
        }
    }
}
