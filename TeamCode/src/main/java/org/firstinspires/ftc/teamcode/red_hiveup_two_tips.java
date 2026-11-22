
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "Red - Hive Up - Two Tips")
public class red_hiveup_two_tips extends OpMode {

    // =========================================
    // APRILTAG
    // =========================================

    private static final int APRILTAG_PIPELINE = OpModeStorage.Alliance.RED.aprilTagPipeline;
    private static final long MAX_RESULT_AGE_MS = 200;
    private static final double TAG_WAIT_MS = 500;

    // =========================================
    // FLYWHEEL
    // =========================================

    private static final double FLYWHEEL_VELOCITY = 1620;
    private static final double ALTERNATE_FLYWHEEL_VELOCITY = 1620;

    // NEW: 20 ticks/second lower for the
    // secondary shot after alternate flower.
    private static final double SECONDARY_FLYWHEEL_VELOCITY = 1600;

    private static final double FLYWHEEL_P = 270;
    private static final double FLYWHEEL_I = 0;
    private static final double FLYWHEEL_D = 0;
    private static final double FLYWHEEL_F = 12.3;

    private static final double FLYWHEEL_WAIT_MS = 3000;
    private static final double VELOCITY_TOLERANCE = 20;
    private static final double VELOCITY_STABLE_MS = 200;

    // =========================================
    // SHOOTING
    // =========================================

    private static final double SHOOT_DURATION_MS = 1500;
    private static final double SHOOT_SERVO_DELAY_MS = 150;

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_PICKUP_POWER = 0.5;
    private static final double TRANSFER_SHOOT_POWER = 0.60;

    // =========================================
    // DRIVING
    // =========================================

    private static final double APPROACH_SPEED = 30.0;
    private static final double CORNER_PICKUP_SPEED = 15.0;
    private static final double PICKUP_DWELL_MS = 500;

    private static final double ROTATE_TIMEOUT_MS = 3000;
    private static final double APPROACH_TIMEOUT_MS = 4000;
    private static final double CORNER_PICKUP_TIMEOUT_MS = 4000;
    private static final double RETURN_TIMEOUT_MS = 5000;
    private static final double PARK_TIMEOUT_MS = 8000;

    private static final double ALTERNATE_TRAVEL_TIMEOUT_MS = 10000;
    private static final double ALTERNATE_TRAVEL_SPEED = 45.0;
    private static final double ALTERNATE_SETTLE_MS = 1000;

    // =========================================
    // AUTONOMOUS TIMING
    // =========================================

    private static final double AUTONOMOUS_LENGTH_MS = 30000;
    private static final double ALTERNATE_PARK_RESERVE_MS = 3000;
    private static final double ALTERNATE_RESHOOT_BUDGET_MS = 4000;
    private static final double ALTERNATE_FLOWER_BUDGET_MS = 4500;
    private static final double ALTERNATE_CLEARANCE_BUDGET_MS = 1500;

    // =========================================
    // ALTERNATE FLOWER
    // =========================================

    private static final int ALTERNATE_FLOWER_PICKUP_CYCLES = 3;

    private static final double ALTERNATE_FLOWER_SPEED = 20.0;

    private static final double ALTERNATE_CLEARANCE_TIMEOUT_MS = 2000;
    private static final double ALTERNATE_FLOWER_APPROACH_TIMEOUT_MS = 2500;
    private static final double ALTERNATE_FLOWER_IN_TIMEOUT_MS = 1500;
    private static final double ALTERNATE_FLOWER_OUT_TIMEOUT_MS = 2000;
    private static final double ALTERNATE_FLOWER_RECOVERY_TIMEOUT_MS = 1800;
    private static final double ALTERNATE_FLOWER_EXIT_TOLERANCE = 2.0;

    // =========================================
    // SECONDARY SHOT
    // =========================================

    private static final double SECONDARY_TRAVEL_TIMEOUT_MS = 3000;
    private static final double SECONDARY_FLYWHEEL_WAIT_MS = 750;
    private static final double SECONDARY_SHOOT_DURATION_MS = 1000;

    // =========================================
    // PRIMARY FLOWER
    // =========================================

    private static final int FLOWER_PICKUP_CYCLES = 4;

    private static final double FLOWER_APPROACH_SPEED = 30.0;
    private static final double FLOWER_SWEEP_SPEED = 15.0;
    private static final double FLOWER_EXIT_SPEED = 20.0;

    private static final double FLOWER_APPROACH_TIMEOUT_MS = 5000;
    private static final double FLOWER_SWEEP_TIMEOUT_MS = 2000;
    private static final double FLOWER_EXIT_TIMEOUT_MS = 2000;

    // =========================================
    // SERVO POSITIONS
    // =========================================

    // 0.18 = UP / STORED
    // 0.69 = DOWN / OUT

    private static final double INTAKE_SERVO_IN = 0.18;
    private static final double INTAKE_SERVO_OUT = 0.69;

    private static final double SHOOTER_SERVO_CLOSED = 0.88;
    private static final double SHOOTER_SERVO_OPEN = 0.5;

    private static final double SHOOTER_SERVO2_CLOSED = 0.5;
    private static final double SHOOTER_SERVO2_OPEN = 0.88;

    // =========================================
    // POSES - LATEST TUNED VALUES
    // =========================================

    private final Pose shootPose =
            new Pose(56, 7.75, Math.toRadians(270));

    private final Pose rotatePose =
            new Pose(56, 7.75, Math.toRadians(180));

    private final Pose pickupApproachPose =
            new Pose(24, 7.75, Math.toRadians(185));

    private final Pose finalCornerPose =
            new Pose(8, 7.75, Math.toRadians(185));

    private final Pose returnControlPose =
            new Pose(28, 11.75, Math.toRadians(270));

    private final Pose alternateShootPose =
            new Pose(59, 132, Math.toRadians(88));

    private final Pose alternateFlowerClearancePose =
            new Pose(59, 117, Math.toRadians(88));

    // Alternate flower OUT.
    private final Pose alternateFlowerStartPose =
            new Pose(48, 122, Math.toRadians(90));

    // Alternate flower IN.
    private final Pose alternateFlowerEndPose =
            new Pose(48, 126.5, Math.toRadians(90));

    // Latest secondary shooting position.
    private final Pose secondaryShootPose =
            new Pose(33.0, 122.0, Math.toRadians(120));

    // Primary flower.
    private final Pose flowerStartPose =
            new Pose(19, 47, Math.toRadians(180));

    private final Pose flowerInPose =
            new Pose(12.75, 47, Math.toRadians(180));

    private final Pose flowerOutPose =
            new Pose(19, 47, Math.toRadians(180));

    private final Pose flowerExitPose =
            new Pose(22, 47, Math.toRadians(180));

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
    // TIMERS AND TRACKING
    // =========================================

    private final Timer stateTimer = new Timer();
    private final Timer velocityTimer = new Timer();

    private boolean velocityStableTracking = false;

    private boolean secondTagSeen = false;
    private int secondTagId = -1;
    private long arrivalFrameTimestamp = -1;

    private boolean alternateTagSeen = false;
    private int alternateTagId = -1;
    private long alternateArrivalFrameTimestamp = -1;

    private String secondShotStatus = "Not checked";

    private int flowerCyclesCompleted = 0;
    private int alternateFlowerCyclesCompleted = 0;
    private boolean alternateFlowerCompleted = false;

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
        CORNER_PICKUP,
        FINISH_PICKUP,

        RETURN_TO_SHOOT,
        WAIT_SECOND_FLYWHEEL,
        SHOOT_SECOND,

        DRIVE_TO_FLOWER,
        FLOWER_SWEEP_IN,
        FLOWER_SWEEP_OUT,
        FLOWER_EXIT,

        DRIVE_TO_ALTERNATE_SHOT,
        WAIT_ALTERNATE_TAG,
        SHOOT_ALTERNATE,

        ALT_FLOWER_CLEARANCE,
        ALT_FLOWER_APPROACH,
        ALT_FLOWER_PICKUP_IN,
        ALT_FLOWER_PICKUP_OUT,
        ALT_FLOWER_RECOVERY,

        ALT_DRIVE_TO_SECONDARY_SHOT,
        ALT_WAIT_SECONDARY_FLYWHEEL,
        ALT_SHOOT_SECONDARY,

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

        intake_servo.setPosition(INTAKE_SERVO_IN);
        closeShooter();

        follower = Constants.create(hardwareMap);
        follower.setPose(shootPose);

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Intake", "UP / STORED");
        telemetry.addData("Start X", shootPose.x());
        telemetry.addData("Start Y", shootPose.y());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Configured field start (in)", "X %.2f, Y %.2f",
                shootPose.x(), shootPose.y());

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()
                || result.getStaleness() > MAX_RESULT_AGE_MS) {
            telemetry.addData("Limelight", "Waiting for a fresh AprilTag frame");
        } else {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            boolean poseShown = false;
            if (tags != null) {
                for (LLResultTypes.FiducialResult tag : tags) {
                    Pose3D pose = tag.getRobotPoseTargetSpace();
                    if (pose == null) {
                        continue;
                    }

                    Position offset = pose.getPosition().toUnit(DistanceUnit.INCH);
                    // For a vertical tag, target-space X and Z are the floor-plane axes.
                    telemetry.addData("Robot from tag " + tag.getFiducialId() + " (in)",
                            "X (sideways) %+.2f, Y (out from tag) %+.2f", offset.x, offset.z);
                    poseShown = true;
                }
            }
            if (!poseShown) {
                telemetry.addData("Limelight", "No AprilTag pose available");
            }
        }
        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        OpModeStorage.beginAutonomous(OpModeStorage.Alliance.RED);

        timeoutCount = 0;
        lastTimeout = "None";

        secondTagSeen = false;
        secondTagId = -1;
        arrivalFrameTimestamp = -1;

        alternateTagSeen = false;
        alternateTagId = -1;
        alternateArrivalFrameTimestamp = -1;

        secondShotStatus = "Not checked";

        flowerCyclesCompleted = 0;
        alternateFlowerCyclesCompleted = 0;
        alternateFlowerCompleted = false;

        resetRuntime();

        intake_servo.setPosition(INTAKE_SERVO_IN);
        closeShooter();
        stopFeeding();

        flywheel_motor.setVelocity(FLYWHEEL_VELOCITY);

        setState(AutoState.WAIT_FIRST_FLYWHEEL);
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        follower.update();

        // UPDATED:
        // Flywheel velocity selected by state.
        if (isFlywheelEnabled()) {
            flywheel_motor.setVelocity(flywheelTargetVelocity());
        } else {
            flywheel_motor.setPower(0);
        }

        switch (state) {

            // =================================
            // FIRST SHOOT
            // =================================

            case WAIT_FIRST_FLYWHEEL:

                stopFeeding();
                closeShooter();

                if (flywheelReady()) {

                    beginShooting(AutoState.SHOOT_FIRST);

                } else if (stateTimer.milliseconds()
                        >= FLYWHEEL_WAIT_MS) {

                    recordTimeout();
                    beginShooting(AutoState.SHOOT_FIRST);
                }
                break;

            case SHOOT_FIRST:

                runShooting();

                if (stateTimer.milliseconds()
                        >= SHOOT_DURATION_MS) {

                    stopFeeding();
                    closeShooter();

                    follower.hold(rotatePose);
                    setState(AutoState.ROTATE_TO_PICKUP);
                }
                break;

            // =================================
            // ROTATE AND CORNER PICKUP
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

                    followLine(
                            pickupApproachPose,
                            APPROACH_SPEED
                    );

                    setState(AutoState.DRIVE_TO_PICKUP);
                }
                break;

            case DRIVE_TO_PICKUP:

                startIntake();

                if (pathFinishedOrTimedOut(
                        APPROACH_TIMEOUT_MS)) {

                    followLine(
                            finalCornerPose,
                            CORNER_PICKUP_SPEED
                    );

                    setState(AutoState.CORNER_PICKUP);
                }
                break;

            case CORNER_PICKUP:

                startIntake();

                if (pathFinishedOrTimedOut(
                        CORNER_PICKUP_TIMEOUT_MS)) {

                    setState(AutoState.FINISH_PICKUP);
                }
                break;

            case FINISH_PICKUP:

                startIntake();

                if (stateTimer.milliseconds()
                        >= PICKUP_DWELL_MS) {

                    stopFeeding();

                    Pose current = follower.pose();

                    Path returnPath =
                            curve(
                                    current,
                                    returnControlPose,
                                    shootPose
                            ).linear(
                                    current,
                                    shootPose
                            );

                    follower.follow(returnPath);

                    setState(AutoState.RETURN_TO_SHOOT);
                }
                break;

            // =================================
            // PRIMARY SECOND SHOT
            // =================================

            case RETURN_TO_SHOOT:

                stopFeeding();
                closeShooter();

                if (pathFinishedOrTimedOut(
                        RETURN_TIMEOUT_MS)) {

                    secondTagSeen = false;
                    secondTagId = -1;

                    secondShotStatus =
                            "Searching for AprilTag";

                    arrivalFrameTimestamp =
                            latestFrameTimestamp();

                    setState(AutoState.WAIT_SECOND_FLYWHEEL);
                }
                break;

            case WAIT_SECOND_FLYWHEEL:

                stopFeeding();
                closeShooter();

                if (!secondTagSeen
                        && secondAprilTagVisible()) {

                    secondTagSeen = true;
                    secondShotStatus = "AprilTag detected";
                }

                boolean ready = flywheelReady();

                if (!secondTagSeen) {

                    if (stateTimer.milliseconds()
                            >= TAG_WAIT_MS) {

                        secondShotStatus =
                                "No primary tag - alternate side";

                        beginAlternateTravel();
                    }

                } else if (ready) {

                    secondShotStatus = "Primary second shot";
                    beginShooting(AutoState.SHOOT_SECOND);

                } else if (stateTimer.milliseconds()
                        >= FLYWHEEL_WAIT_MS) {

                    recordTimeout();

                    secondShotStatus =
                            "Primary shot - flywheel timeout";

                    beginShooting(AutoState.SHOOT_SECOND);
                }
                break;

            case SHOOT_SECOND:

                runShooting();

                if (stateTimer.milliseconds()
                        >= SHOOT_DURATION_MS) {

                    secondShotStatus =
                            "Primary second shot complete";

                    beginFlowerApproach();
                }
                break;

            // =================================
            // PRIMARY FLOWER - FOUR CYCLES
            // =================================

            case DRIVE_TO_FLOWER:

                intake_servo.setPosition(INTAKE_SERVO_OUT);
                stopFeeding();
                closeShooter();

                if (!follower.isBusy()) {

                    flowerCyclesCompleted = 0;
                    beginFlowerSweepIn();

                } else if (stateTimer.milliseconds()
                        >= FLOWER_APPROACH_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();
                    beginFlowerPark();
                }
                break;

            case FLOWER_SWEEP_IN:

                intake_servo.setPosition(INTAKE_SERVO_OUT);
                startIntake();
                closeShooter();

                if (!follower.isBusy()) {

                    beginFlowerSweepOut();

                } else if (stateTimer.milliseconds()
                        >= FLOWER_SWEEP_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();
                    beginFlowerExit();
                }
                break;

            case FLOWER_SWEEP_OUT:

                intake_servo.setPosition(INTAKE_SERVO_OUT);
                startIntake();
                closeShooter();

                if (!follower.isBusy()) {

                    flowerCyclesCompleted++;

                    if (flowerCyclesCompleted
                            < FLOWER_PICKUP_CYCLES) {

                        beginFlowerSweepIn();

                    } else {

                        beginFlowerExit();
                    }

                } else if (stateTimer.milliseconds()
                        >= FLOWER_SWEEP_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();
                    beginFlowerExit();
                }
                break;

            case FLOWER_EXIT:

                intake_servo.setPosition(INTAKE_SERVO_OUT);
                startIntake();
                closeShooter();

                if (!follower.isBusy()) {

                    beginFlowerPark();

                } else if (stateTimer.milliseconds()
                        >= FLOWER_EXIT_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();
                    beginFlowerPark();
                }
                break;

            // =================================
            // FIRST ALTERNATE SHOT
            // =================================

            case DRIVE_TO_ALTERNATE_SHOT:

                stopFeeding();
                closeShooter();

                if (!follower.isBusy()) {

                    alternateTagSeen = false;
                    alternateTagId = -1;

                    alternateArrivalFrameTimestamp =
                            latestFrameTimestamp();

                    secondShotStatus =
                            "Checking alternate AprilTag";

                    setState(AutoState.WAIT_ALTERNATE_TAG);

                } else if (stateTimer.milliseconds()
                        >= ALTERNATE_TRAVEL_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();

                    secondShotStatus =
                            "Alternate travel timeout";

                    beginAlternatePark();
                }
                break;

            case WAIT_ALTERNATE_TAG:

                stopFeeding();
                closeShooter();

                if (!alternateTagSeen
                        && alternateAprilTagVisible()) {

                    alternateTagSeen = true;

                    secondShotStatus =
                            "Alternate tag detected";
                }

                if (stateTimer.milliseconds()
                        >= ALTERNATE_SETTLE_MS) {

                    if (alternateTagSeen) {

                        beginShooting(AutoState.SHOOT_ALTERNATE);

                    } else {

                        secondShotStatus =
                                "No alternate tag - parking";

                        beginAlternatePark();
                    }
                }
                break;

            case SHOOT_ALTERNATE:

                runShooting();

                if (stateTimer.milliseconds()
                        >= SHOOT_DURATION_MS) {

                    secondShotStatus =
                            "First alternate shot complete";

                    if (remainingAutoMs()
                            >= ALTERNATE_PARK_RESERVE_MS
                            + ALTERNATE_FLOWER_BUDGET_MS
                            + ALTERNATE_CLEARANCE_BUDGET_MS) {

                        beginAlternateFlowerClearance();

                    } else {

                        secondShotStatus =
                                "Not enough time for flower - park";

                        beginAlternatePark();
                    }
                }
                break;

            // =================================
            // ALTERNATE FLOWER CLEARANCE
            // =================================

            case ALT_FLOWER_CLEARANCE:

                // Arm UP during retreat.
                intake_servo.setPosition(INTAKE_SERVO_IN);

                stopFeeding();
                closeShooter();

                if (!follower.isBusy()) {

                    beginAlternateFlowerApproach();

                } else if (stateTimer.milliseconds()
                        >= ALTERNATE_CLEARANCE_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();

                    secondShotStatus =
                            "Clearance timeout - park";

                    beginAlternatePark();
                }
                break;

            // =================================
            // ALTERNATE FLOWER APPROACH
            // =================================

            case ALT_FLOWER_APPROACH:

                // Arm DOWN.
                intake_servo.setPosition(INTAKE_SERVO_OUT);

                stopFeeding();
                closeShooter();

                if (!follower.isBusy()) {

                    beginAlternateFlowerIn();

                } else if (stateTimer.milliseconds()
                        >= ALTERNATE_FLOWER_APPROACH_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();

                    secondShotStatus =
                            "Alternate approach timeout";

                    stopFeeding();
                    setState(AutoState.DONE);
                }
                break;

            // =================================
            // ALTERNATE FLOWER IN
            // =================================

            case ALT_FLOWER_PICKUP_IN:

                // Keep intake arm DOWN.
                intake_servo.setPosition(INTAKE_SERVO_OUT);

                startIntake();
                closeShooter();

                if (!follower.isBusy()) {

                    beginAlternateFlowerOut();

                } else if (stateTimer.milliseconds()
                        >= ALTERNATE_FLOWER_IN_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();

                    secondShotStatus =
                            "Flower IN timeout - moving OUT";

                    beginAlternateFlowerOut();
                }
                break;

            // =================================
            // ALTERNATE FLOWER OUT
            // =================================

            case ALT_FLOWER_PICKUP_OUT:

                // Keep arm DOWN and motors running.
                intake_servo.setPosition(INTAKE_SERVO_OUT);

                startIntake();
                closeShooter();

                if (!follower.isBusy()) {

                    if (alternateFlowerAtApproach()) {

                        alternateFlowerCyclesCompleted++;

                        if (alternateFlowerCyclesCompleted
                                < ALTERNATE_FLOWER_PICKUP_CYCLES) {

                            beginAlternateFlowerIn();

                        } else {

                            // After THREE completed cycles,
                            // raise the arm at the OUT pose.
                            finishAlternateFlowerPickup();
                        }

                    } else {

                        beginAlternateFlowerRecovery();
                    }

                } else if (stateTimer.milliseconds()
                        >= ALTERNATE_FLOWER_OUT_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();

                    beginAlternateFlowerRecovery();
                }
                break;

            // =================================
            // ALTERNATE FLOWER RECOVERY
            // =================================

            case ALT_FLOWER_RECOVERY:

                // Never lift bar inside the flower.
                intake_servo.setPosition(INTAKE_SERVO_OUT);

                startIntake();
                closeShooter();

                if (alternateFlowerAtApproach()
                        && !follower.isBusy()) {

                    secondShotStatus =
                            "Flower recovered - parking";

                    stopFeeding();
                    intake_servo.setPosition(INTAKE_SERVO_IN);

                    alternateFlowerCompleted = false;

                    beginAlternatePark();

                } else if (stateTimer.milliseconds()
                        >= ALTERNATE_FLOWER_RECOVERY_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();

                    stopFeeding();
                    closeShooter();
                    flywheel_motor.setPower(0);

                    secondShotStatus =
                            "Flower exit failed - arm held DOWN";

                    setState(AutoState.DONE);
                }
                break;

            // =================================
            // SECONDARY SHOT TRAVEL
            // =================================

            case ALT_DRIVE_TO_SECONDARY_SHOT:

                stopFeeding();
                closeShooter();

                intake_servo.setPosition(INTAKE_SERVO_IN);

                // Flywheel target is now 1600
                // during this travel state.
                if (!follower.isBusy()) {

                    secondShotStatus =
                            "At secondary shoot position";

                    setState(AutoState.ALT_WAIT_SECONDARY_FLYWHEEL);

                } else if (stateTimer.milliseconds()
                        >= SECONDARY_TRAVEL_TIMEOUT_MS) {

                    recordTimeout();
                    follower.stop();

                    secondShotStatus =
                            "Secondary shooting travel timeout";

                    beginAlternatePark();
                }
                break;

            // =================================
            // SECONDARY FLYWHEEL CHECK
            // =================================

            case ALT_WAIT_SECONDARY_FLYWHEEL:

                stopFeeding();
                closeShooter();

                intake_servo.setPosition(INTAKE_SERVO_IN);

                // flywheelReady() now checks 1600
                // specifically for secondary shooting.
                if (flywheelReady()
                        || stateTimer.milliseconds()
                        >= SECONDARY_FLYWHEEL_WAIT_MS) {

                    secondShotStatus =
                            "Shooting secondary position";

                    beginShooting(AutoState.ALT_SHOOT_SECONDARY);
                }
                break;

            // =================================
            // SECONDARY SHOOT - ONE SECOND
            // =================================

            case ALT_SHOOT_SECONDARY:

                // 1600 flywheel target.
                // Intake OFF.
                // Transfer ON after servo delay.
                runShooting();

                if (stateTimer.milliseconds()
                        >= SECONDARY_SHOOT_DURATION_MS) {

                    secondShotStatus =
                            "Secondary shot complete";

                    beginAlternatePark();
                }
                break;

            // =================================
            // PARKING
            // =================================

            case PARK:

                if (pathFinishedOrTimedOut(
                        PARK_TIMEOUT_MS)) {

                    stopFeeding();
                    closeShooter();
                    flywheel_motor.setPower(0);

                    setState(AutoState.DONE);
                }
                break;

            case DONE:

                stopFeeding();
                closeShooter();
                flywheel_motor.setPower(0);

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

        telemetry.addData(
                "Primary Flower Cycles",
                flowerCyclesCompleted
        );

        telemetry.addData(
                "Primary Flower Target",
                FLOWER_PICKUP_CYCLES
        );

        telemetry.addData(
                "Alternate Flower Cycles Completed",
                alternateFlowerCyclesCompleted
        );

        telemetry.addData(
                "Alternate Flower Cycles Target",
                ALTERNATE_FLOWER_PICKUP_CYCLES
        );

        telemetry.addData(
                "Alternate Flower Complete",
                alternateFlowerCompleted
        );

        telemetry.addData(
                "Alternate Flower IN Target",
                "(48, 126.5, 90)"
        );

        telemetry.addData(
                "Alternate Flower OUT Target",
                "(48, 122, 90)"
        );

        telemetry.addData(
                "At Alternate Flower OUT",
                alternateFlowerAtApproach()
        );

        telemetry.addData(
                "Secondary Shot Target",
                "(33.0, 122.0, 118)"
        );

        telemetry.addData(
                "Intake Servo",
                intake_servo.getPosition()
        );

        telemetry.addData(
                "Intake Motor",
                intake_motor.getPower()
        );

        telemetry.addData(
                "Transfer Motor",
                transfer_motor.getPower()
        );

        // UPDATED: telemetry uses the actual
        // state-specific velocity target.
        double flywheelTarget =
                isFlywheelEnabled()
                        ? flywheelTargetVelocity()
                        : 0;

        double flywheelActual =
                flywheel_motor.getVelocity();

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

        telemetry.addData(
                "Follower Busy",
                follower.isBusy()
        );

        telemetry.addData(
                "Limelight Connected",
                limelight.isConnected()
        );

        telemetry.addData(
                "Second Shot Status",
                secondShotStatus
        );

        telemetry.addData(
                "Primary AprilTag ID",
                secondTagId
        );

        telemetry.addData(
                "Alternate AprilTag ID",
                alternateTagId
        );

        telemetry.addData(
                "Auto Remaining (ms)",
                Math.max(0, remainingAutoMs())
        );

        telemetry.addData(
                "Alternate Flower Start Minimum (ms)",
                ALTERNATE_PARK_RESERVE_MS
                        + ALTERNATE_FLOWER_BUDGET_MS
                        + ALTERNATE_CLEARANCE_BUDGET_MS
        );

        telemetry.addData("Timeout Count", timeoutCount);
        telemetry.addData("Last Timeout", lastTimeout);

        telemetry.update();
    }

    // =========================================
    // PATH HELPER
    // =========================================

    private void followLine(Pose target, double speed) {

        Pose current = follower.pose();

        Path path =
                line(current, target)
                        .linear(current, target)
                        .with(
                                Constants.foresightConfig
                                        .maxVelocityConstraint
                                        .at(speed)
                        );

        follower.follow(path);
    }

    // =========================================
    // FLYWHEEL TARGET SELECTION
    // =========================================

    // NEW: Secondary states use 1600.
    private boolean isSecondaryShotState() {

        return state == AutoState.ALT_DRIVE_TO_SECONDARY_SHOT
                || state == AutoState.ALT_WAIT_SECONDARY_FLYWHEEL
                || state == AutoState.ALT_SHOOT_SECONDARY;
    }

    // Centralizes the target for motor control,
    // velocity readiness, and telemetry.
    private double flywheelTargetVelocity() {

        if (isSecondaryShotState()) {
            return SECONDARY_FLYWHEEL_VELOCITY;
        }

        return isAlternateFlywheelState()
                ? ALTERNATE_FLYWHEEL_VELOCITY
                : FLYWHEEL_VELOCITY;
    }

    private boolean isAlternateFlywheelState() {

        return state == AutoState.DRIVE_TO_ALTERNATE_SHOT
                || state == AutoState.WAIT_ALTERNATE_TAG
                || state == AutoState.SHOOT_ALTERNATE
                || state == AutoState.ALT_DRIVE_TO_SECONDARY_SHOT
                || state == AutoState.ALT_WAIT_SECONDARY_FLYWHEEL
                || state == AutoState.ALT_SHOOT_SECONDARY;
    }

    private boolean isFlywheelEnabled() {

        return state != AutoState.DRIVE_TO_FLOWER
                && state != AutoState.FLOWER_SWEEP_IN
                && state != AutoState.FLOWER_SWEEP_OUT
                && state != AutoState.FLOWER_EXIT
                && state != AutoState.ALT_FLOWER_CLEARANCE
                && state != AutoState.ALT_FLOWER_APPROACH
                && state != AutoState.ALT_FLOWER_PICKUP_IN
                && state != AutoState.ALT_FLOWER_PICKUP_OUT
                && state != AutoState.ALT_FLOWER_RECOVERY
                && state != AutoState.PARK
                && state != AutoState.DONE;
    }

    // =========================================
    // LIMELIGHT HELPERS
    // =========================================

    private long latestFrameTimestamp() {

        LLResult result = limelight.getLatestResult();

        return result != null
                ? result.getControlHubTimeStamp()
                : -1;
    }

    private boolean secondAprilTagVisible() {

        if (!limelight.isConnected()) return false;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return false;

        if (result.getPipelineIndex()
                != APRILTAG_PIPELINE) return false;

        if (result.getStaleness()
                > MAX_RESULT_AGE_MS) return false;

        if (result.getControlHubTimeStamp()
                <= arrivalFrameTimestamp) return false;

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        if (tags == null || tags.isEmpty()) return false;

        secondTagId = tags.get(0).getFiducialId();

        return true;
    }

    private boolean alternateAprilTagVisible() {

        if (!limelight.isConnected()) return false;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return false;

        if (result.getPipelineIndex()
                != APRILTAG_PIPELINE) return false;

        if (result.getStaleness()
                > MAX_RESULT_AGE_MS) return false;

        if (result.getControlHubTimeStamp()
                <= alternateArrivalFrameTimestamp) return false;

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        if (tags == null || tags.isEmpty()) return false;

        alternateTagId = tags.get(0).getFiducialId();

        return true;
    }

    // =========================================
    // PRIMARY FLOWER HELPERS
    // =========================================

    private void beginFlowerApproach() {

        stopFeeding();
        closeShooter();
        flywheel_motor.setPower(0);

        intake_servo.setPosition(INTAKE_SERVO_OUT);
        flowerCyclesCompleted = 0;

        followLine(
                flowerStartPose,
                FLOWER_APPROACH_SPEED
        );

        setState(AutoState.DRIVE_TO_FLOWER);
    }

    private void beginFlowerSweepIn() {

        intake_servo.setPosition(INTAKE_SERVO_OUT);

        startIntake();
        closeShooter();

        followLine(
                flowerInPose,
                FLOWER_SWEEP_SPEED
        );

        setState(AutoState.FLOWER_SWEEP_IN);
    }

    private void beginFlowerSweepOut() {

        intake_servo.setPosition(INTAKE_SERVO_OUT);

        startIntake();
        closeShooter();

        followLine(
                flowerOutPose,
                FLOWER_SWEEP_SPEED
        );

        setState(AutoState.FLOWER_SWEEP_OUT);
    }

    private void beginFlowerExit() {

        intake_servo.setPosition(INTAKE_SERVO_OUT);

        startIntake();
        closeShooter();

        followLine(
                flowerExitPose,
                FLOWER_EXIT_SPEED
        );

        setState(AutoState.FLOWER_EXIT);
    }

    private void beginFlowerPark() {

        stopFeeding();
        closeShooter();

        intake_servo.setPosition(INTAKE_SERVO_IN);
        flywheel_motor.setPower(0);

        Pose current = follower.pose();

        Path path =
                line(current, parkPose)
                        .linear(current, parkPose);

        follower.follow(path);

        setState(AutoState.PARK);
    }

    // =========================================
    // TIME MANAGEMENT
    // =========================================

    private double remainingAutoMs() {

        return AUTONOMOUS_LENGTH_MS
                - getRuntime() * 1000.0;
    }

    // =========================================
    // ALTERNATE FLOWER POSITION CHECK
    // =========================================

    private boolean alternateFlowerAtApproach() {

        Pose current = follower.pose();

        double dx =
                current.x() - alternateFlowerStartPose.x();

        double dy =
                current.y() - alternateFlowerStartPose.y();

        return Math.hypot(dx, dy)
                <= ALTERNATE_FLOWER_EXIT_TOLERANCE;
    }

    // =========================================
    // ALTERNATE FLOWER HELPERS
    // =========================================

    private void beginAlternateFlowerClearance() {

        stopFeeding();
        closeShooter();
        flywheel_motor.setPower(0);

        // Arm UP during retreat.
        intake_servo.setPosition(INTAKE_SERVO_IN);

        alternateFlowerCyclesCompleted = 0;
        alternateFlowerCompleted = false;

        followLine(
                alternateFlowerClearancePose,
                ALTERNATE_FLOWER_SPEED
        );

        secondShotStatus =
                "Backing away before arm deployment";

        setState(AutoState.ALT_FLOWER_CLEARANCE);
    }

    private void beginAlternateFlowerApproach() {

        stopFeeding();
        closeShooter();
        flywheel_motor.setPower(0);

        // Lower only after clearance.
        intake_servo.setPosition(INTAKE_SERVO_OUT);

        followLine(
                alternateFlowerStartPose,
                ALTERNATE_FLOWER_SPEED
        );

        secondShotStatus =
                "Approaching alternate flower";

        setState(AutoState.ALT_FLOWER_APPROACH);
    }

    private void beginAlternateFlowerIn() {

        intake_servo.setPosition(INTAKE_SERVO_OUT);

        startIntake();
        closeShooter();
        flywheel_motor.setPower(0);

        followLine(
                alternateFlowerEndPose,
                ALTERNATE_FLOWER_SPEED
        );

        secondShotStatus =
                "Alternate flower moving IN";

        setState(AutoState.ALT_FLOWER_PICKUP_IN);
    }

    private void beginAlternateFlowerOut() {

        // Keep arm DOWN throughout OUT movement.
        intake_servo.setPosition(INTAKE_SERVO_OUT);

        startIntake();
        closeShooter();
        flywheel_motor.setPower(0);

        followLine(
                alternateFlowerStartPose,
                ALTERNATE_FLOWER_SPEED
        );

        secondShotStatus =
                "Alternate flower moving OUT";

        setState(AutoState.ALT_FLOWER_PICKUP_OUT);
    }

    private void beginAlternateFlowerRecovery() {

        // Do not raise arm during recovery.
        intake_servo.setPosition(INTAKE_SERVO_OUT);

        startIntake();
        closeShooter();
        flywheel_motor.setPower(0);

        followLine(
                alternateFlowerStartPose,
                ALTERNATE_FLOWER_SPEED
        );

        secondShotStatus =
                "Recovering alternate flower exit";

        setState(AutoState.ALT_FLOWER_RECOVERY);
    }

    // =========================================
    // FINISH THREE ALTERNATE FLOWER CYCLES
    // =========================================

    private void finishAlternateFlowerPickup() {

        // Three IN/OUT cycles completed.
        // Robot is back at (48, 122, 90).
        // NOW raise the intake.

        stopFeeding();
        closeShooter();

        intake_servo.setPosition(INTAKE_SERVO_IN);

        alternateFlowerCompleted = true;

        secondShotStatus =
                "Three alternate flower cycles completed";

        beginSecondaryShotTravel();
    }

    // =========================================
    // SECONDARY SHOT TRAVEL
    // =========================================

    private void beginSecondaryShotTravel() {

        stopFeeding();
        closeShooter();

        intake_servo.setPosition(INTAKE_SERVO_IN);

        followLine(
                secondaryShootPose,
                ALTERNATE_TRAVEL_SPEED
        );

        // Once the state changes, the flywheel
        // will start targeting 1600.
        secondShotStatus =
                "Driving to secondary shooting pose";

        setState(AutoState.ALT_DRIVE_TO_SECONDARY_SHOT);
    }

    // =========================================
    // INITIAL ALTERNATE TRAVEL
    // =========================================

    private void beginAlternateTravel() {

        stopFeeding();
        closeShooter();

        intake_servo.setPosition(INTAKE_SERVO_IN);

        followLine(
                alternateShootPose,
                ALTERNATE_TRAVEL_SPEED
        );

        secondShotStatus =
                "Traveling to alternate shooting position";

        setState(AutoState.DRIVE_TO_ALTERNATE_SHOT);
    }

    // =========================================
    // PARK
    // =========================================

    private void beginAlternatePark() {

        stopFeeding();
        closeShooter();

        intake_servo.setPosition(INTAKE_SERVO_IN);
        flywheel_motor.setPower(0);

        Pose current = follower.pose();

        Path path =
                line(current, parkPose)
                        .linear(current, parkPose);

        follower.follow(path);

        setState(AutoState.PARK);
    }

    // =========================================
    // STATE MANAGEMENT
    // =========================================

    private void setState(AutoState newState) {

        state = newState;
        stateTimer.reset();
        velocityStableTracking = false;
    }

    private boolean pathFinishedOrTimedOut(double timeoutMs) {

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

    private void recordTimeout() {

        timeoutCount++;
        lastTimeout = state.name();
    }

    // =========================================
    // FLYWHEEL READY CHECK
    // =========================================

    private boolean flywheelReady() {

        double actualVelocity =
                flywheel_motor.getVelocity();

        // UPDATED: use the active target.
        // Secondary shooting = 1600.
        // All previous shooting = 1620.
        double velocityError = Math.abs(
                flywheelTargetVelocity() - actualVelocity
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
    // SHOOTING
    // =========================================

    private void beginShooting(AutoState shootingState) {

        stopFeeding();
        openShooter();

        setState(shootingState);
    }

    private void runShooting() {

        openShooter();

        // Intake ON for first shot only.
        // Intake OFF for all later shots.
        boolean firstShooting =
                state == AutoState.SHOOT_FIRST;

        if (!firstShooting) {
            intake_motor.setPower(0);
        }

        if (stateTimer.milliseconds()
                >= SHOOT_SERVO_DELAY_MS) {

            intake_motor.setPower(
                    firstShooting ? INTAKE_POWER : 0
            );

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
    // INTAKE / TRANSFER
    // =========================================

    private void startIntake() {

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

        if (follower != null) {

            OpModeStorage.autonomousEndPose =
                    follower.pose();

            follower.stop();
        }

        if (limelight != null) {
            limelight.stop();
        }

        if (flywheel_motor != null) {
            flywheel_motor.setPower(0);
        }

        if (intake_motor != null) {
            intake_motor.setPower(0);
        }

        if (transfer_motor != null) {
            transfer_motor.setPower(0);
        }

        // Avoid raising bar if stopped
        // inside the alternate flower.
        if (intake_servo != null) {

            if (alternateFlowerAtApproach()
                    || !isInsideAlternateFlowerState()) {

                intake_servo.setPosition(INTAKE_SERVO_IN);
            }
        }

        if (shooter_servo != null
                && shooter_servo2 != null) {

            closeShooter();
        }
    }

    private boolean isInsideAlternateFlowerState() {

        return state == AutoState.ALT_FLOWER_PICKUP_IN
                || state == AutoState.ALT_FLOWER_PICKUP_OUT
                || state == AutoState.ALT_FLOWER_RECOVERY;
    }
}
