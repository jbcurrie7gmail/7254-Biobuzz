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

@Autonomous(name = "Blue - Hive Up")
public class blue_hiveup extends OpMode {

    // =========================================
    // LIMELIGHT CONSTANTS
    // =========================================

    private static final int APRILTAG_PIPELINE = 1;
    private static final long MAX_RESULT_AGE_MS = 200;
    private static final double TAG_WAIT_MS = 500;

    // =========================================
    // FLYWHEEL CONSTANTS
    // =========================================

    private static final double FLYWHEEL_VELOCITY = 1620;

    private static final double FLYWHEEL_P = 270;
    private static final double FLYWHEEL_I = 0;
    private static final double FLYWHEEL_D = 0;
    private static final double FLYWHEEL_F = 12.3;

    private static final double FLYWHEEL_WAIT_MS = 3000;
    private static final double VELOCITY_TOLERANCE = 20;
    private static final double VELOCITY_STABLE_MS = 200;

    // =========================================
    // SHOOTING CONSTANTS
    // =========================================

    private static final double SHOOT_DURATION_MS = 1500;
    private static final double SHOOT_SERVO_DELAY_MS = 150;

    // =========================================
    // INTAKE CONSTANTS
    // =========================================

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_PICKUP_POWER = 0.5;
    private static final double TRANSFER_SHOOT_POWER = 0.60;

    // =========================================
    // DRIVING SPEED CONSTANTS
    // =========================================

    private static final double APPROACH_SPEED = 30.0;
    private static final double SLOW_PICKUP_SPEED = 15.0;

    // NEW: Final corner pickup speed
    private static final double FINAL_CORNER_SPEED = 15.0;

    private static final double PICKUP_DWELL_MS = 500;

    // =========================================
    // AUTONOMOUS TIMEOUT CONSTANTS
    // =========================================

    private static final double ROTATE_TIMEOUT_MS = 3000;
    private static final double APPROACH_TIMEOUT_MS = 4000;
    private static final double PICKUP_TIMEOUT_MS = 3000;

    // NEW: Final corner movement timeout
    private static final double FINAL_CORNER_TIMEOUT_MS = 2000;

    private static final double RETURN_TIMEOUT_MS = 4000;
    private static final double STRAFE_TIMEOUT_MS = 3000;
    private static final double PARK_TIMEOUT_MS = 8000;

    // =========================================
    // SERVO POSITIONS
    // =========================================

    private static final double INTAKE_SERVO_IN = 0.18;

    private static final double SHOOTER_SERVO_CLOSED = 0.88;
    private static final double SHOOTER_SERVO_OPEN = 0.5;

    private static final double SHOOTER_SERVO2_CLOSED = 0.5;
    private static final double SHOOTER_SERVO2_OPEN = 0.88;

    // =========================================
    // FIELD POSES
    // =========================================

    private final Pose shootPose =
            new Pose(56, 7.75, Math.toRadians(270));

    private final Pose rotatePose =
            new Pose(56, 7.75, Math.toRadians(180));

    private final Pose pickupApproachPose =
            new Pose(24, 7.75, Math.toRadians(185));

    private final Pose pickupPose =
            new Pose(12, 7.75, Math.toRadians(180));

    // NEW:
    // Move another 2 inches into the corner
    // while rotating back to 180 degrees.

    private final Pose finalCornerPose =
            new Pose(3, 7.75, Math.toRadians(180));

    private final Pose strafePose =
            new Pose(32, 24, Math.toRadians(270));

    private final Pose parkPose =
            new Pose(12, 93, Math.toRadians(270));

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
    private final Timer velocityTimer = new Timer();

    private boolean velocityStableTracking = false;

    // =========================================
    // LIMELIGHT TRACKING
    // =========================================

    private boolean secondTagSeen = false;

    private int secondTagId = -1;

    private long arrivalFrameTimestamp = -1;

    private String secondShotStatus = "Not checked";

    // =========================================
    // TIMEOUT TRACKING
    // =========================================

    private int timeoutCount = 0;
    private String lastTimeout = "None";

    // =========================================
    // AUTONOMOUS STATES
    // =========================================

    private enum AutoState {

        WAIT_FIRST_FLYWHEEL,
        SHOOT_FIRST,

        ROTATE_TO_PICKUP,
        DRIVE_TO_PICKUP,
        SLOW_PICKUP,

        // NEW STATE
        FINAL_CORNER_PICKUP,

        FINISH_PICKUP,

        RETURN_TO_SHOOT,
        WAIT_SECOND_FLYWHEEL,
        SHOOT_SECOND,

        STRAFE_LEFT,
        PARK,
        DONE
    }

    private AutoState state =
            AutoState.WAIT_FIRST_FLYWHEEL;

    // =========================================
    // INITIALIZATION
    // =========================================

    @Override
    public void init() {

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

        intake_servo.setPosition(INTAKE_SERVO_IN);

        closeShooter();

        // =====================================
        // PEDRO PATHING INITIALIZATION
        // =====================================

        follower = Constants.create(hardwareMap);

        follower.setPose(shootPose);

        // =====================================
        // LIMELIGHT INITIALIZATION
        // =====================================

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        limelight.start();

        telemetry.addData("Status", "Initialized");

        telemetry.addData(
                "Limelight Connected",
                limelight.isConnected());

        telemetry.addData(
                "Limelight Pipeline",
                APRILTAG_PIPELINE);

        telemetry.addData(
                "AprilTag Detection", "Any ID");

        telemetry.addData("Start X", shootPose.x());
        telemetry.addData("Start Y", shootPose.y());
        telemetry.addData("Start Heading", 270);

        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        OpModeStorage.autonomousEndPose = null;

        timeoutCount = 0;
        lastTimeout = "None";

        secondTagSeen = false;
        secondTagId = -1;
        arrivalFrameTimestamp = -1;
        secondShotStatus = "Not checked";

        // Start flywheel immediately
        flywheel_motor.setVelocity(FLYWHEEL_VELOCITY);

        setState(AutoState.WAIT_FIRST_FLYWHEEL);
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        // Update Pedro Pathing once per loop
        follower.update();

        // =====================================
        // FLYWHEEL CONTROL
        // =====================================

        // Maintain flywheel velocity until second
        // shooting is completed or skipped.

        if (isFlywheelEnabled()) {

            flywheel_motor.setVelocity(
                    FLYWHEEL_VELOCITY);

        } else {

            flywheel_motor.setPower(0);
        }

        switch (state) {

            // =================================
            // FIRST FLYWHEEL SPINUP
            // =================================

            case WAIT_FIRST_FLYWHEEL:

                stopFeeding();
                closeShooter();

                // First shooting does not
                // require AprilTag detection.

                if (flywheelReady()) {

                    beginShooting(AutoState.SHOOT_FIRST);

                } else if (stateTimer.milliseconds()
                        >= FLYWHEEL_WAIT_MS) {

                    recordTimeout();

                    beginShooting(AutoState.SHOOT_FIRST);
                }

                break;

            // =================================
            // FIRST SHOOTING SEQUENCE
            // =================================

            case SHOOT_FIRST:

                runShooting();

                if (stateTimer.milliseconds()
                        >= SHOOT_DURATION_MS) {

                    stopFeeding();
                    closeShooter();

                    // Keep flywheel running.

                    follower.hold(rotatePose);

                    setState(AutoState.ROTATE_TO_PICKUP);
                }

                break;

            // =================================
            // ROTATE TO 180 DEGREES
            // =================================

            case ROTATE_TO_PICKUP:

                double headingError = Math.atan2(
                        Math.sin(
                                Math.toRadians(180)
                                        - follower.pose().heading()
                        ),
                        Math.cos(
                                Math.toRadians(180)
                                        - follower.pose().heading()
                        )
                );

                boolean rotationComplete =
                        Math.abs(headingError)
                                < Math.toRadians(5);

                boolean rotationTimedOut =
                        stateTimer.milliseconds()
                                >= ROTATE_TIMEOUT_MS;

                if (rotationComplete || rotationTimedOut) {

                    if (rotationTimedOut && !rotationComplete) {

                        recordTimeout();
                        follower.stop();
                    }

                    startIntake();

                    Pose startPose = follower.pose();

                    Path approach =
                            line(startPose, pickupApproachPose)
                                    .linear(
                                            startPose,
                                            pickupApproachPose
                                    )
                                    .with(
                                            Constants.foresightConfig
                                                    .maxVelocityConstraint
                                                    .at(APPROACH_SPEED)
                                    );

                    follower.follow(approach);

                    setState(AutoState.DRIVE_TO_PICKUP);
                }

                break;

            // =================================
            // DRIVE TOWARD CORNER
            // =================================

            case DRIVE_TO_PICKUP:

                startIntake();

                if (pathFinishedOrTimedOut(
                        APPROACH_TIMEOUT_MS)) {

                    Pose startPose = follower.pose();

                    Path slowApproach =
                            line(startPose, pickupPose)
                                    .linear(
                                            startPose,
                                            pickupPose
                                    )
                                    .with(
                                            Constants.foresightConfig
                                                    .maxVelocityConstraint
                                                    .at(SLOW_PICKUP_SPEED)
                                    );

                    follower.follow(slowApproach);

                    setState(AutoState.SLOW_PICKUP);
                }

                break;

            // =================================
            // SLOW CORNER PICKUP
            // =================================

            case SLOW_PICKUP:

                startIntake();

                if (pathFinishedOrTimedOut(
                        PICKUP_TIMEOUT_MS)) {

                    // NEW:
                    // After reaching pickupPose,
                    // move farther into the corner.
                    //
                    // X: 5 -> 3
                    // Y: 6 -> 6
                    // Heading: 235 -> 180 degrees

                    Pose startPose = follower.pose();

                    Path finalCornerPath =
                            line(startPose, finalCornerPose)
                                    .linear(
                                            startPose,
                                            finalCornerPose
                                    )
                                    .with(
                                            Constants.foresightConfig
                                                    .maxVelocityConstraint
                                                    .at(FINAL_CORNER_SPEED)
                                    );

                    follower.follow(finalCornerPath);

                    setState(
                            AutoState.FINAL_CORNER_PICKUP
                    );
                }

                break;

            // =================================
            // NEW: FINAL CORNER PICKUP
            // =================================

            case FINAL_CORNER_PICKUP:

                // Keep intake and transfer running
                // while moving and rotating.

                startIntake();

                if (pathFinishedOrTimedOut(
                        FINAL_CORNER_TIMEOUT_MS)) {

                    // Begin pickup dwell after
                    // final corner movement.

                    setState(AutoState.FINISH_PICKUP);
                }

                break;

            // =================================
            // FINISH PICKUP
            // =================================

            case FINISH_PICKUP:

                // Continue collecting balls
                // for the pickup dwell period.

                startIntake();

                if (stateTimer.milliseconds()
                        >= PICKUP_DWELL_MS) {

                    stopFeeding();

                    Pose startPose = follower.pose();

                    Path returnPath =
                            line(startPose, shootPose)
                                    .linear(
                                            startPose,
                                            shootPose
                                    );

                    follower.follow(returnPath);

                    setState(AutoState.RETURN_TO_SHOOT);
                }

                break;

            // =================================
            // RETURN TO SHOOTING POSITION
            // =================================

            case RETURN_TO_SHOOT:

                stopFeeding();
                closeShooter();

                if (pathFinishedOrTimedOut(
                        RETURN_TIMEOUT_MS)) {

                    // Reset AprilTag detection.

                    secondTagSeen = false;
                    secondTagId = -1;

                    secondShotStatus =
                            "Searching for AprilTag";

                    // Capture camera timestamp at arrival.

                    LLResult arrivalResult =
                            limelight.getLatestResult();

                    arrivalFrameTimestamp =
                            arrivalResult != null
                                    ? arrivalResult.getControlHubTimeStamp()
                                    : -1;

                    setState(
                            AutoState.WAIT_SECOND_FLYWHEEL
                    );
                }

                break;

            // =================================
            // SECOND SHOOTING READINESS
            // REQUIRE APRILTAG DETECTION
            // =================================

            case WAIT_SECOND_FLYWHEEL:

                stopFeeding();
                closeShooter();

                if (!secondTagSeen
                        && secondAprilTagVisible()) {

                    secondTagSeen = true;

                    secondShotStatus = "AprilTag detected";
                }

                boolean secondFlywheelReady =
                        flywheelReady();

                if (!secondTagSeen) {

                    // Skip shooting if no AprilTag
                    // is detected within TAG_WAIT_MS.

                    if (stateTimer.milliseconds()
                            >= TAG_WAIT_MS) {

                        secondShotStatus =
                                "Skipped - no AprilTag";

                        // Shut off flywheel and move
                        // directly toward parking.

                        beginStrafe();
                    }

                } else if (secondFlywheelReady) {

                    secondShotStatus =
                            "Shooting - AprilTag detected";

                    beginShooting(AutoState.SHOOT_SECOND);

                } else if (stateTimer.milliseconds()
                        >= FLYWHEEL_WAIT_MS) {

                    // Preserve flywheel timeout behavior,
                    // but only when AprilTag was detected.

                    recordTimeout();

                    secondShotStatus =
                            "Shooting - flywheel timeout";

                    beginShooting(AutoState.SHOOT_SECOND);
                }

                break;

            // =================================
            // SECOND SHOOTING SEQUENCE
            // =================================

            case SHOOT_SECOND:

                runShooting();

                if (stateTimer.milliseconds()
                        >= SHOOT_DURATION_MS) {

                    secondShotStatus =
                            "Second shot completed";

                    // Stop feeding, close shooter,
                    // shut off flywheel and strafe.

                    beginStrafe();
                }

                break;

            // =================================
            // STRAFE NEGATIVE X
            // =================================

            case STRAFE_LEFT:

                // Flywheel remains OFF.

                if (pathFinishedOrTimedOut(
                        STRAFE_TIMEOUT_MS)) {

                    Pose startPose = follower.pose();

                    Path parkingPath =
                            line(startPose, parkPose)
                                    .linear(
                                            startPose,
                                            parkPose
                                    );

                    follower.follow(parkingPath);

                    setState(AutoState.PARK);
                }

                break;

            // =================================
            // PARK
            // =================================

            case PARK:

                // Flywheel remains OFF.

                if (pathFinishedOrTimedOut(
                        PARK_TIMEOUT_MS)) {

                    stopFeeding();
                    closeShooter();

                    flywheel_motor.setPower(0);

                    setState(AutoState.DONE);
                }

                break;

            // =================================
            // DONE
            // =================================

            case DONE:

                flywheel_motor.setPower(0);

                stopFeeding();

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
                Math.toDegrees(robotPose.heading())
        );

        // =====================================
        // FINAL CORNER TELEMETRY
        // =====================================

        if (state == AutoState.FINAL_CORNER_PICKUP) {

            telemetry.addData(
                    "Final Corner Target X",
                    finalCornerPose.x()
            );

            telemetry.addData(
                    "Final Corner Target Y",
                    finalCornerPose.y()
            );

            telemetry.addData(
                    "Final Corner Target Heading",
                    Math.toDegrees(finalCornerPose.heading())
            );

            telemetry.addData(
                    "Final Corner Speed",
                    FINAL_CORNER_SPEED
            );
        }

        // =====================================
        // FLYWHEEL TELEMETRY
        // =====================================

        boolean flywheelEnabled = isFlywheelEnabled();

        double flywheelTarget =
                flywheelEnabled ? FLYWHEEL_VELOCITY : 0;

        double flywheelActual =
                flywheel_motor.getVelocity();

        telemetry.addData(
                "Flywheel Enabled",
                flywheelEnabled
        );

        telemetry.addData(
                "Flywheel Target",
                flywheelTarget
        );

        telemetry.addData(
                "Flywheel Actual",
                flywheelActual
        );

        telemetry.addData(
                "Flywheel Error",
                flywheelTarget - flywheelActual
        );

        telemetry.addData(
                "State Time (ms)",
                stateTimer.milliseconds()
        );

        // =====================================
        // PEDRO PATHING TELEMETRY
        // =====================================

        telemetry.addData(
                "Follower Busy",
                follower.isBusy()
        );

        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        // =====================================
        // LIMELIGHT TELEMETRY
        // =====================================

        telemetry.addData(
                "Limelight Connected",
                limelight.isConnected()
        );

        telemetry.addData(
                "Limelight Running",
                limelight.isRunning()
        );

        telemetry.addData(
                "Limelight Pipeline",
                APRILTAG_PIPELINE
        );

        telemetry.addData(
                "Second AprilTag Seen",
                secondTagSeen
        );

        telemetry.addData(
                "Second AprilTag ID",
                secondTagId
        );

        telemetry.addData(
                "Second Shot",
                secondShotStatus
        );

        if (state == AutoState.WAIT_SECOND_FLYWHEEL
                && !secondTagSeen) {

            telemetry.addData(
                    "AprilTag Wait Remaining (ms)",
                    Math.max(
                            0,
                            TAG_WAIT_MS
                                    - stateTimer.milliseconds()
                    )
            );
        }

        // =====================================
        // TIMEOUT TELEMETRY
        // =====================================

        telemetry.addData(
                "Timeout Count",
                timeoutCount
        );

        telemetry.addData(
                "Last Timeout",
                lastTimeout
        );

        telemetry.update();
    }

    // =========================================
    // FLYWHEEL ENABLE CHECK
    // =========================================

    private boolean isFlywheelEnabled() {

        return state != AutoState.STRAFE_LEFT
                && state != AutoState.PARK
                && state != AutoState.DONE;
    }

    // =========================================
    // APRILTAG DETECTION
    // =========================================

    private boolean secondAprilTagVisible() {

        if (!limelight.isConnected()) {

            return false;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {

            return false;
        }

        if (result.getPipelineIndex()
                != APRILTAG_PIPELINE) {

            return false;
        }

        if (result.getStaleness()
                > MAX_RESULT_AGE_MS) {

            return false;
        }

        // Reject camera results received
        // before returning to shooting position.

        if (result.getControlHubTimeStamp()
                <= arrivalFrameTimestamp) {

            return false;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        if (tags == null || tags.isEmpty()) {

            return false;
        }

        // Accept any AprilTag ID.

        secondTagId = tags.get(0).getFiducialId();

        return true;
    }

    // =========================================
    // BEGIN STRAFE
    // =========================================

    // Called after successful second shooting
    // OR when second shooting is skipped.

    private void beginStrafe() {

        stopFeeding();

        closeShooter();

        // Turn off flywheel before moving.
        flywheel_motor.setPower(0);

        Pose startPose = follower.pose();

        Path strafePath =
                line(startPose, strafePose)
                        .linear(
                                startPose,
                                strafePose
                        );

        follower.follow(strafePath);

        setState(AutoState.STRAFE_LEFT);
    }

    // =========================================
    // STATE MANAGEMENT
    // =========================================

    private void setState(AutoState newState) {

        state = newState;

        stateTimer.reset();

        velocityStableTracking = false;
    }

    // =========================================
    // PATH COMPLETION / TIMEOUT HANDLING
    // =========================================

    private boolean pathFinishedOrTimedOut(
            double timeoutMs) {

        if (!follower.isBusy()) {

            return true;
        }

        if (stateTimer.milliseconds() >= timeoutMs) {

            recordTimeout();

            follower.stop();

            return true;
        }

        return false;
    }

    // =========================================
    // TIMEOUT TELEMETRY
    // =========================================

    private void recordTimeout() {

        timeoutCount++;

        lastTimeout = state.name();
    }

    // =========================================
    // FLYWHEEL VELOCITY CHECK
    // =========================================

    private boolean flywheelReady() {

        double actualVelocity =
                flywheel_motor.getVelocity();

        double velocityError = Math.abs(
                FLYWHEEL_VELOCITY - actualVelocity
        );

        if (velocityError <= VELOCITY_TOLERANCE) {

            if (!velocityStableTracking) {

                velocityTimer.reset();

                velocityStableTracking = true;
            }

            return velocityTimer.milliseconds()
                    >= VELOCITY_STABLE_MS;

        } else {

            velocityStableTracking = false;

            return false;
        }
    }

    // =========================================
    // SHOOTING CONTROL
    // =========================================

    private void beginShooting(AutoState shootingState) {

        stopFeeding();

        openShooter();

        setState(shootingState);
    }

    private void runShooting() {

        openShooter();

        if (stateTimer.milliseconds()
                >= SHOOT_SERVO_DELAY_MS) {

            intake_motor.setPower(INTAKE_POWER);

            transfer_motor.setPower(
                    TRANSFER_SHOOT_POWER
            );

        } else {

            stopFeeding();
        }
    }

    private void openShooter() {

        shooter_servo.setPosition(
                SHOOTER_SERVO_OPEN
        );

        shooter_servo2.setPosition(
                SHOOTER_SERVO2_OPEN
        );
    }

    private void closeShooter() {

        shooter_servo.setPosition(
                SHOOTER_SERVO_CLOSED
        );

        shooter_servo2.setPosition(
                SHOOTER_SERVO2_CLOSED
        );
    }

    // =========================================
    // INTAKE / TRANSFER MOTOR CONTROL
    // =========================================

    private void startIntake() {

        // Intake servo remains IN at 0.18.

        intake_motor.setPower(INTAKE_POWER);

        transfer_motor.setPower(
                TRANSFER_PICKUP_POWER
        );
    }

    private void stopFeeding() {

        intake_motor.setPower(0);

        transfer_motor.setPower(0);
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        // Save actual ending position for TeleOp.

        if (follower != null) {

            OpModeStorage.autonomousEndPose =
                    follower.pose();

            follower.stop();
        }

        // Stop Limelight polling.

        if (limelight != null) {

            limelight.stop();
        }

        // Stop all motors.

        if (flywheel_motor != null) {

            flywheel_motor.setPower(0);
        }

        if (intake_motor != null) {

            intake_motor.setPower(0);
        }

        if (transfer_motor != null) {

            transfer_motor.setPower(0);
        }

        if (shooter_servo != null
                && shooter_servo2 != null) {

            closeShooter();
        }
    }
}