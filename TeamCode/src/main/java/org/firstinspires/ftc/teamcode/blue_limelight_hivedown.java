
package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "Blue - Hive Down")
public class blue_limelight_hivedown extends OpMode {

    // =========================================
    // LIMELIGHT SETTINGS
    // =========================================

    // BLUE APRILTAG PIPELINE
    private static final int APRILTAG_PIPELINE = 1;

    // Confirm detection for 150 milliseconds
    private static final double TAG_CONFIRM_MS = 150;

    // Begin feeding 3 seconds after first detection
    private static final double TAG_SHOOT_DELAY_MS = 3000;

    // Ignore old Limelight results
    private static final long MAX_RESULT_AGE_MS = 200;

    // Optional fallback if no AprilTag is detected
    // Disabled so shooting requires AprilTag detection
    private static final boolean ENABLE_TIME_FALLBACK = false;

    private static final double FALLBACK_SHOOT_MS = 17000;

    // =========================================
    // SHOOTING CONSTANTS
    // =========================================

    private static final double SHOOT_DURATION_MS = 1500;
    private static final double SHOOT_SERVO_DELAY_MS = 150;

    // =========================================
    // FLYWHEEL CONSTANTS
    // =========================================

    private static final double FLYWHEEL_VELOCITY = 1620;

    private static final double FLYWHEEL_P = 270;
    private static final double FLYWHEEL_I = 0;
    private static final double FLYWHEEL_D = 0;
    private static final double FLYWHEEL_F = 12.3;

    // =========================================
    // MOTOR POWERS
    // =========================================

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_SHOOT_POWER = 0.75;

    // =========================================
    // MOVEMENT TIMEOUTS
    // =========================================

    private static final double CLEAR_FLOWER_TIMEOUT_MS = 3000;
    private static final double PARK_TIMEOUT_MS = 7000;

    // =========================================
    // SERVO POSITIONS
    // =========================================

    // Intake stays IN for entire autonomous
    private static final double INTAKE_SERVO_IN = 0.2;

    private static final double SHOOTER_SERVO_CLOSED = 0.88;
    private static final double SHOOTER_SERVO_OPEN = 0.5;

    private static final double SHOOTER_SERVO2_CLOSED = 0.5;
    private static final double SHOOTER_SERVO2_OPEN = 0.88;

    // =========================================
    // FIELD POSES
    // =========================================

    // Starting against upper wall
    private final Pose startPose =
            new Pose(60, 136.25, Math.toRadians(90));

    // Move 10 inches in negative Y to clear FLOWER
    private final Pose clearFlowerPose =
            new Pose(60, 126.25, Math.toRadians(90));

    // Final parking position
    private final Pose parkPose =
            new Pose(13, 120, Math.toRadians(90));

    // =========================================
    // HARDWARE
    // =========================================

    private Follower follower;
    private Limelight3A limelight;

    private DcMotorEx intake_motor;
    private DcMotorEx transfer_motor;
    private DcMotorEx flywheel_motor;

    private Servo intake_servo;
    private Servo shooter_servo;
    private Servo shooter_servo2;

    // =========================================
    // TIMERS
    // =========================================

    private final Timer stateTimer = new Timer();
    private final Timer autonomousTimer = new Timer();
    private final Timer tagDetectionTimer = new Timer();

    // =========================================
    // VISION TRACKING
    // =========================================

    private boolean trackingTag = false;
    private boolean tagConfirmed = false;
    private boolean tagVisibleNow = false;

    // Count unique Limelight frames
    private int validTagFrames = 0;

    private long lastTagFrameTimestamp = -1;
    private long currentTagFrameTimestamp = -1;

    private int detectedTagId = -1;
    private int triggerTagId = -1;

    private String shootTrigger = "Waiting";

    // =========================================
    // TIMEOUT TRACKING
    // =========================================

    private int timeoutCount = 0;
    private String lastTimeout = "None";

    // =========================================
    // AUTONOMOUS STATES
    // =========================================

    private enum AutoState {
        WAIT_FOR_TAG,
        TAG_DELAY,
        SHOOT,
        CLEAR_FLOWER,
        PARK,
        DONE
    }

    private AutoState state = AutoState.WAIT_FOR_TAG;

    // =========================================
    // INITIALIZATION
    // =========================================

    @Override
    public void init() {

        // Hardware initialization
        intake_motor = hardwareMap.get(
                DcMotorEx.class, "intake_motor");

        transfer_motor = hardwareMap.get(
                DcMotorEx.class, "transfer_motor");

        flywheel_motor = hardwareMap.get(
                DcMotorEx.class, "flywheel_motor");

        intake_servo = hardwareMap.get(
                Servo.class, "intake_servo");

        shooter_servo = hardwareMap.get(
                Servo.class, "shooter_servo");

        shooter_servo2 = hardwareMap.get(
                Servo.class, "shooter_servo2");

        // =====================================
        // MOTOR INITIALIZATION
        // =====================================

        intake_motor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer_motor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel_motor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        // Flywheel PIDF
        flywheel_motor.setVelocityPIDFCoefficients(
                FLYWHEEL_P,
                FLYWHEEL_I,
                FLYWHEEL_D,
                FLYWHEEL_F
        );

        intake_motor.setPower(0);
        transfer_motor.setPower(0);
        flywheel_motor.setPower(0);

        // =====================================
        // SERVO INITIALIZATION
        // =====================================

        // Intake servo remains IN
        intake_servo.setPosition(INTAKE_SERVO_IN);

        closeShooter();

        // =====================================
        // PEDRO PATHING INITIALIZATION
        // =====================================

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);

        // Keep endpoint holding enabled
        follower.holdEnd.set(true);

        // =====================================
        // LIMELIGHT INITIALIZATION
        // =====================================

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        // SELECT BLUE APRILTAG PIPELINE 1
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start X", startPose.x());
        telemetry.addData("Start Y", startPose.y());
        telemetry.addData(
                "Start Heading",
                Math.toDegrees(startPose.heading()));

        telemetry.addData(
                "Limelight Pipeline",
                APRILTAG_PIPELINE);

        telemetry.addData("AprilTag Detection", "Any ID");

        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        // Clear previous autonomous ending pose
        OpModeStorage.autonomousEndPose = null;

        timeoutCount = 0;
        lastTimeout = "None";

        trackingTag = false;
        tagConfirmed = false;
        tagVisibleNow = false;

        validTagFrames = 0;

        lastTagFrameTimestamp = -1;
        currentTagFrameTimestamp = -1;

        detectedTagId = -1;
        triggerTagId = -1;

        shootTrigger = "Waiting";

        autonomousTimer.reset();

        // Start flywheel immediately
        flywheel_motor.setVelocity(FLYWHEEL_VELOCITY);

        setState(AutoState.WAIT_FOR_TAG);
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        // Update Pedro Pathing
        follower.update();

        // Maintain flywheel velocity until done
        if (state != AutoState.DONE) {
            flywheel_motor.setVelocity(FLYWHEEL_VELOCITY);
        }

        switch (state) {

            // =================================
            // WAIT FOR APRILTAG DETECTION
            // =================================

            case WAIT_FOR_TAG:

                stopFeeding();
                closeShooter();

                tagVisibleNow = anyAprilTagVisible();

                if (tagVisibleNow) {

                    // Start timer on first detection
                    if (!trackingTag) {

                        trackingTag = true;

                        tagDetectionTimer.reset();

                        validTagFrames = 0;

                        lastTagFrameTimestamp = -1;
                    }

                    // Count distinct camera frames
                    if (currentTagFrameTimestamp !=
                            lastTagFrameTimestamp) {

                        validTagFrames++;

                        lastTagFrameTimestamp =
                                currentTagFrameTimestamp;
                    }

                    // Require stable detection
                    if (tagDetectionTimer.milliseconds()
                            >= TAG_CONFIRM_MS &&
                            validTagFrames >= 2) {

                        tagConfirmed = true;

                        triggerTagId = detectedTagId;

                        shootTrigger = "AprilTag";

                        // Keep detection timer running
                        // to measure the full 3 seconds
                        // from first sighting
                        setState(AutoState.TAG_DELAY);
                    }

                } else {

                    // Restart confirmation if tag lost
                    trackingTag = false;

                    validTagFrames = 0;

                    lastTagFrameTimestamp = -1;
                }

                // Optional timed backup
                if (state == AutoState.WAIT_FOR_TAG &&
                        ENABLE_TIME_FALLBACK &&
                        autonomousTimer.milliseconds()
                                >= FALLBACK_SHOOT_MS) {

                    shootTrigger = "Timed Fallback";

                    beginShooting();
                }

                break;

            // =================================
            // THREE-SECOND DELAY
            // =================================

            case TAG_DELAY:

                stopFeeding();
                closeShooter();

                // Begin opening shooter servos
                // 150 ms before feeding starts
                if (tagDetectionTimer.milliseconds()
                        >= TAG_SHOOT_DELAY_MS
                        - SHOOT_SERVO_DELAY_MS) {

                    beginShooting();
                }

                break;

            // =================================
            // SHOOT FOUR BALLS
            // =================================

            case SHOOT:

                openShooter();

                // Delay to allow servos to open
                if (stateTimer.milliseconds()
                        >= SHOOT_SERVO_DELAY_MS) {

                    intake_motor.setPower(INTAKE_POWER);

                    transfer_motor.setPower(
                            TRANSFER_SHOOT_POWER);

                } else {

                    stopFeeding();
                }

                // Finish shooting sequence
                if (stateTimer.milliseconds()
                        >= SHOOT_DURATION_MS) {

                    stopFeeding();

                    closeShooter();

                    // Move negative Y to clear FLOWER
                    followTo(clearFlowerPose);

                    setState(AutoState.CLEAR_FLOWER);
                }

                break;

            // =================================
            // CLEAR FLOWER
            // =================================

            case CLEAR_FLOWER:

                if (pathFinishedOrTimedOut(
                        CLEAR_FLOWER_TIMEOUT_MS)) {

                    // Follow parking path from
                    // current estimated position
                    followTo(parkPose);

                    setState(AutoState.PARK);
                }

                break;

            // =================================
            // PARK
            // =================================

            case PARK:

                if (pathFinishedOrTimedOut(
                        PARK_TIMEOUT_MS)) {

                    finishAuto();

                    setState(AutoState.DONE);
                }

                break;

            // =================================
            // DONE
            // =================================

            case DONE:

                flywheel_motor.setPower(0);

                stopFeeding();

                // Keep follower updating so
                // the robot holds its parking pose
                break;
        }

        // =====================================
        // TELEMETRY
        // =====================================

        Pose robotPose = follower.pose();

        telemetry.addData("Auto State", state);

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());

        telemetry.addData(
                "Robot Heading",
                Math.toDegrees(robotPose.heading()));

        telemetry.addData(
                "Autonomous Time (ms)",
                autonomousTimer.milliseconds());

        telemetry.addData(
                "State Time (ms)",
                stateTimer.milliseconds());

        // Limelight telemetry
        telemetry.addData(
                "Limelight Connected",
                limelight.isConnected());

        telemetry.addData(
                "Limelight Running",
                limelight.isRunning());

        telemetry.addData(
                "Limelight Pipeline",
                APRILTAG_PIPELINE);

        telemetry.addData(
                "AprilTag Visible",
                tagVisibleNow);

        telemetry.addData(
                "Tag Confirmed",
                tagConfirmed);

        telemetry.addData(
                "Detected Tag ID",
                detectedTagId);

        telemetry.addData(
                "Trigger Tag ID",
                triggerTagId);

        telemetry.addData(
                "Valid Tag Frames",
                validTagFrames);

        telemetry.addData(
                "Shoot Trigger",
                shootTrigger);

        if (state == AutoState.TAG_DELAY) {

            telemetry.addData(
                    "Time Until Feeding (ms)",
                    Math.max(
                            0,
                            TAG_SHOOT_DELAY_MS
                                    - tagDetectionTimer.milliseconds()
                    )
            );
        }

        telemetry.addData(
                "Flywheel Target",
                FLYWHEEL_VELOCITY);

        telemetry.addData(
                "Flywheel Actual",
                flywheel_motor.getVelocity());

        telemetry.addData(
                "Flywheel Error",
                FLYWHEEL_VELOCITY
                        - flywheel_motor.getVelocity());

        telemetry.addData(
                "Follower Busy",
                follower.isBusy());

        telemetry.addData(
                "Follower Mode",
                follower.mode());

        telemetry.addData(
                "Timeout Count",
                timeoutCount);

        telemetry.addData(
                "Last Timeout",
                lastTimeout);

        telemetry.update();
    }

    // =========================================
    // APRILTAG DETECTION - PIPELINE 1
    // =========================================

    private boolean anyAprilTagVisible() {

        LLResult result = limelight.getLatestResult();

        // No valid result
        if (result == null || !result.isValid()) {
            return false;
        }

        // Only use blue pipeline 1
        if (result.getPipelineIndex()
                != APRILTAG_PIPELINE) {
            return false;
        }

        // Reject stale camera results
        if (result.getStaleness()
                > MAX_RESULT_AGE_MS) {
            return false;
        }

        // Get all detected AprilTags
        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        if (tags == null || tags.isEmpty()) {
            return false;
        }

        // Accept any detected AprilTag
        detectedTagId =
                tags.get(0).getFiducialId();

        // Timestamp identifies camera frame
        currentTagFrameTimestamp =
                result.getControlHubTimeStamp();

        return true;
    }

    // =========================================
    // BEGIN SHOOTING
    // =========================================

    private void beginShooting() {

        stopFeeding();

        openShooter();

        setState(AutoState.SHOOT);
    }

    // =========================================
    // FOLLOW PATH
    // =========================================

    private void followTo(Pose targetPose) {

        Pose currentPose = follower.pose();

        Path path =
                line(currentPose, targetPose)
                        .linear(currentPose, targetPose);

        follower.follow(path);
    }

    // =========================================
    // PATH TIMEOUT
    // =========================================

    private boolean pathFinishedOrTimedOut(
            double timeoutMs) {

        if (!follower.isBusy()) {
            return true;
        }

        if (stateTimer.milliseconds()
                >= timeoutMs) {

            recordTimeout();

            follower.stop();

            return true;
        }

        return false;
    }

    // =========================================
    // FINISH AUTONOMOUS
    // =========================================

    private void finishAuto() {

        // Save actual ending pose for TeleOp
        Pose finalPose = follower.pose();

        OpModeStorage.autonomousEndPose = finalPose;

        // Continue holding ending position
        follower.hold(finalPose);

        flywheel_motor.setPower(0);

        stopFeeding();

        closeShooter();
    }

    // =========================================
    // STATE MANAGEMENT
    // =========================================

    private void setState(AutoState newState) {

        state = newState;

        stateTimer.reset();
    }

    // =========================================
    // TIMEOUT TELEMETRY
    // =========================================

    private void recordTimeout() {

        timeoutCount++;

        lastTimeout = state.name();
    }

    // =========================================
    // SHOOTER CONTROL
    // =========================================

    private void openShooter() {

        shooter_servo.setPosition(
                SHOOTER_SERVO_OPEN);

        shooter_servo2.setPosition(
                SHOOTER_SERVO2_OPEN);
    }

    private void closeShooter() {

        shooter_servo.setPosition(
                SHOOTER_SERVO_CLOSED);

        shooter_servo2.setPosition(
                SHOOTER_SERVO2_CLOSED);
    }

    // =========================================
    // INTAKE / TRANSFER CONTROL
    // =========================================

    private void stopFeeding() {

        intake_motor.setPower(0);

        transfer_motor.setPower(0);
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        // Save actual ending pose for TeleOp
        if (follower != null) {

            OpModeStorage.autonomousEndPose =
                    follower.pose();

            follower.stop();
        }

        // Stop Limelight polling
        if (limelight != null) {
            limelight.stop();
        }

        // Stop all motors
        if (flywheel_motor != null) {
            flywheel_motor.setPower(0);
        }

        if (intake_motor != null) {
            intake_motor.setPower(0);
        }

        if (transfer_motor != null) {
            transfer_motor.setPower(0);
        }

        if (shooter_servo != null &&
                shooter_servo2 != null) {
            closeShooter();
        }
    }
}
