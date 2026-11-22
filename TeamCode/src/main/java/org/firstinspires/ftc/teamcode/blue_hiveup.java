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

@Autonomous(name = "Blue - Hive Up")
public class blue_hiveup extends OpMode {
    private static final int APRILTAG_PIPELINE = OpModeStorage.Alliance.BLUE.aprilTagPipeline;
    private static final long MAX_RESULT_AGE_MS = 200;
    private static final double AUTONOMOUS_LENGTH_MS = 30_000;
    private static final double TAG_WAIT_RESERVE_MS = 6_000;
    private static final double PARK_RESERVE_MS = 3_000;

    private static final double FLYWHEEL_VELOCITY = 1620;

    private static final double FLYWHEEL_P = 270;
    private static final double FLYWHEEL_I = 0;
    private static final double FLYWHEEL_D = 0;
    private static final double FLYWHEEL_F = 12.3;

    private static final double FLYWHEEL_WAIT_MS = 3000;
    private static final double VELOCITY_TOLERANCE = 20;
    private static final double VELOCITY_STABLE_MS = 200;

    private static final double SHOOT_DURATION_MS = 1500;
    private static final double SHOOT_SERVO_DELAY_MS = 150;

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_PICKUP_POWER = 0.5;
    private static final double TRANSFER_SHOOT_POWER = 0.60;

    private static final double APPROACH_SPEED = 30.0;
    private static final double CORNER_PICKUP_SPEED = 15.0;
    private static final double PICKUP_DWELL_MS = 500;

    private static final double ROTATE_TIMEOUT_MS = 3000;
    private static final double APPROACH_TIMEOUT_MS = 4000;
    private static final double CORNER_PICKUP_TIMEOUT_MS = 4000;
    private static final double RETURN_TIMEOUT_MS = 5000;
    private static final double PARK_TIMEOUT_MS = 8000;

    private static final int FLOWER_PICKUP_CYCLES = 4;

    private static final double FLOWER_APPROACH_SPEED = 30.0;
    private static final double FLOWER_SWEEP_SPEED = 15.0;
    private static final double FLOWER_EXIT_SPEED = 20.0;

    private static final double FLOWER_APPROACH_TIMEOUT_MS = 5000;
    private static final double FLOWER_SWEEP_TIMEOUT_MS = 2000;
    private static final double FLOWER_EXIT_TIMEOUT_MS = 2000;
    private static final double FLOWER_PICKUP_BUDGET_MS =
            FLOWER_APPROACH_TIMEOUT_MS + FLOWER_SWEEP_TIMEOUT_MS + FLOWER_EXIT_TIMEOUT_MS;

    private static final double INTAKE_SERVO_IN = 0.18;
    private static final double INTAKE_SERVO_OUT = 0.69;

    private static final double SHOOTER_SERVO_CLOSED = 0.88;
    private static final double SHOOTER_SERVO_OPEN = 0.5;

    private static final double SHOOTER_SERVO2_CLOSED = 0.5;
    private static final double SHOOTER_SERVO2_OPEN = 0.88;

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

    private final Pose flowerStartPose =
            new Pose(19, 47, Math.toRadians(180));

    private final Pose flowerInPose =
            new Pose(12.75, 47, Math.toRadians(180));

    private final Pose flowerOutPose =
            new Pose(19, 47, Math.toRadians(180));

    private final Pose flowerExitPose =
            new Pose(22, 47, Math.toRadians(180));

    private static final double HIVE_CLEARANCE_TOLERANCE = 2.0;

    private final Pose hiveClearancePose =
            new Pose(24, 24, Math.toRadians(270));

    private final Pose parkPose =
            new Pose(12, 93, Math.toRadians(270));

    // Allow travel beyond the clearance line if the follower settles short of it.
    private final Pose flowerClearanceRecoveryPose =
            new Pose(flowerExitPose.x() + 1, flowerExitPose.y(), flowerExitPose.heading());

    private Follower follower;
    private Limelight3A limelight;

    private DcMotorEx intake_motor;
    private DcMotorEx transfer_motor;
    private DcMotorEx flywheel_motor;

    private Servo intake_servo;
    private Servo shooter_servo;
    private Servo shooter_servo2;

    // TIMERS AND TRACKING

    private final Timer stateTimer = new Timer();
    private final Timer velocityTimer = new Timer();

    private boolean velocityStableTracking = false;

    private int secondTagId = -1;
    private int flowerCyclesCompleted;
    private int timeoutCount;
    private String lastTimeout = "None";

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
        PARK_CLEAR_HIVE,
        PARK,
        DONE
    }

    private AutoState state = AutoState.WAIT_FIRST_FLYWHEEL;

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

    @Override
    public void start() {
        OpModeStorage.beginAutonomous(OpModeStorage.Alliance.BLUE);
        timeoutCount = 0;
        lastTimeout = "None";
        secondTagId = -1;
        flowerCyclesCompleted = 0;
        resetRuntime();

        intake_servo.setPosition(INTAKE_SERVO_IN);
        closeShooter();
        stopFeeding();
        flywheel_motor.setVelocity(FLYWHEEL_VELOCITY);
        setState(AutoState.WAIT_FIRST_FLYWHEEL);
    }

    @Override
    public void loop() {
        follower.update();

        double remainingMs = remainingAutoMs();
        if (remainingMs <= PARK_RESERVE_MS
                && state != AutoState.PARK_CLEAR_HIVE
                && state != AutoState.PARK && state != AutoState.DONE) {
            if (isFlowerState() && !flowerIsClear()) {
                if (state != AutoState.FLOWER_EXIT) {
                    beginFlowerExit();
                }
            } else {
                beginPark();
            }
        }

        if (isFlowerState() || state == AutoState.PARK_CLEAR_HIVE
                || state == AutoState.PARK || state == AutoState.DONE) {
            flywheel_motor.setPower(0);
        } else {
            flywheel_motor.setVelocity(FLYWHEEL_VELOCITY);
        }

        switch (state) {
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

            case RETURN_TO_SHOOT:
                stopFeeding();
                closeShooter();

                if (pathFinishedOrTimedOut(RETURN_TIMEOUT_MS)) {
                    // Retain the readiness measured during the return drive.
                    state = AutoState.WAIT_SECOND_FLYWHEEL;
                    stateTimer.reset();
                    checkForSecondShot(remainingMs);
                } else {
                    flywheelReady();
                }
                break;

            case WAIT_SECOND_FLYWHEEL:
                checkForSecondShot(remainingMs);
                break;

            case SHOOT_SECOND:
                runShooting();
                if (stateTimer.milliseconds() >= SHOOT_DURATION_MS) {
                    if (remainingMs >= FLOWER_PICKUP_BUDGET_MS + PARK_RESERVE_MS) {
                        beginFlowerApproach();
                    } else {
                        beginPark();
                    }
                }
                break;

            case DRIVE_TO_FLOWER:
                if (!hasTimeForFlowerSweep(remainingMs)) {
                    beginFlowerExit();
                } else if (!follower.isBusy()) {
                    beginFlowerSweepIn();
                } else if (stateTimer.milliseconds() >= FLOWER_APPROACH_TIMEOUT_MS) {
                    recordTimeout();
                    beginFlowerExit();
                }
                break;

            case FLOWER_SWEEP_IN:
                if (remainingMs <= PARK_RESERVE_MS + FLOWER_EXIT_TIMEOUT_MS) {
                    beginFlowerExit();
                } else if (!follower.isBusy()) {
                    beginFlowerSweepOut();
                } else if (stateTimer.milliseconds() >= FLOWER_SWEEP_TIMEOUT_MS) {
                    recordTimeout();
                    beginFlowerExit();
                }
                break;

            case FLOWER_SWEEP_OUT:
                if (!follower.isBusy()) {
                    flowerCyclesCompleted++;
                    if (flowerCyclesCompleted < FLOWER_PICKUP_CYCLES
                            && hasTimeForFlowerSweep(remainingMs)) {
                        beginFlowerSweepIn();
                    } else {
                        beginFlowerExit();
                    }
                } else if (remainingMs <= PARK_RESERVE_MS + FLOWER_EXIT_TIMEOUT_MS) {
                    beginFlowerExit();
                } else if (stateTimer.milliseconds() >= FLOWER_SWEEP_TIMEOUT_MS) {
                    recordTimeout();
                    beginFlowerExit();
                }
                break;

            case FLOWER_EXIT:
                if (flowerIsClear()) {
                    beginPark();
                } else if (pathFinishedOrTimedOut(FLOWER_EXIT_TIMEOUT_MS)) {
                    followLine(flowerClearanceRecoveryPose, FLOWER_EXIT_SPEED);
                    setState(AutoState.FLOWER_EXIT);
                }
                break;

            case PARK_CLEAR_HIVE:
                Pose current = follower.pose();
                if (!follower.isBusy()
                        && Math.hypot(current.x() - hiveClearancePose.x(),
                                current.y() - hiveClearancePose.y()) <= HIVE_CLEARANCE_TOLERANCE) {
                    // Keep the park leg on the clear side of Flower A.
                    Pose control = new Pose(36, flowerExitPose.y(), parkPose.heading());
                    follower.follow(curve(current, control, parkPose).linear(current, parkPose));
                    setState(AutoState.PARK);
                } else if (stateTimer.milliseconds() >= PARK_TIMEOUT_MS) {
                    recordTimeout();
                    follower.stop();
                    setState(AutoState.DONE);
                } else if (!follower.isBusy()) {
                    follower.follow(line(current, hiveClearancePose).linear(current, hiveClearancePose));
                }
                break;

            case PARK:
                if (pathFinishedOrTimedOut(PARK_TIMEOUT_MS)) {
                    setState(AutoState.DONE);
                }
                break;

            case DONE:
                break;
        }

        Pose pose = follower.pose();
        telemetry.addData("Auto State", state);
        telemetry.addData("Time remaining (s)", "%.2f", remainingMs / 1000.0);
        telemetry.addData("Robot pose", "X %.2f, Y %.2f, heading %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Primary AprilTag ID", secondTagId);
        telemetry.addData("Flywheel velocity", flywheel_motor.getVelocity());
        telemetry.addData("Flower A cycles", "%d / %d", flowerCyclesCompleted, FLOWER_PICKUP_CYCLES);
        telemetry.addData("Timeouts", timeoutCount);
        telemetry.addData("Last timeout", lastTimeout);
        telemetry.update();
    }

    private void checkForSecondShot(double remainingMs) {
        if (secondTagId < 0) {
            if (remainingMs <= TAG_WAIT_RESERVE_MS) {
                beginPark();
                return;
            }
            secondTagId = visibleAprilTagId();
        }

        boolean ready = flywheelReady();
        if (secondTagId < 0) {
            return;
        }
        if (ready) {
            beginShooting(AutoState.SHOOT_SECOND);
        } else if (stateTimer.milliseconds() >= FLYWHEEL_WAIT_MS
                || remainingMs <= PARK_RESERVE_MS + SHOOT_DURATION_MS + SHOOT_SERVO_DELAY_MS) {
            recordTimeout();
            beginShooting(AutoState.SHOOT_SECOND);
        }
    }

    private int visibleAprilTagId() {
        if (!limelight.isConnected()) {
            return -1;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()
                || result.getPipelineIndex() != APRILTAG_PIPELINE
                || result.getStaleness() > MAX_RESULT_AGE_MS) {
            return -1;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        return tags == null || tags.isEmpty() ? -1 : tags.get(0).getFiducialId();
    }

    private boolean hasTimeForFlowerSweep(double remainingMs) {
        return remainingMs > PARK_RESERVE_MS + FLOWER_EXIT_TIMEOUT_MS + FLOWER_SWEEP_TIMEOUT_MS;
    }

    private boolean isFlowerState() {
        return state == AutoState.DRIVE_TO_FLOWER
                || state == AutoState.FLOWER_SWEEP_IN
                || state == AutoState.FLOWER_SWEEP_OUT
                || state == AutoState.FLOWER_EXIT;
    }

    private boolean flowerIsClear() {
        return follower.pose().x() >= flowerExitPose.x();
    }

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

    private void beginPark() {
        stopFeeding();
        closeShooter();
        flywheel_motor.setPower(0);
        intake_servo.setPosition(INTAKE_SERVO_IN);

        Pose current = follower.pose();
        if (state == AutoState.WAIT_FIRST_FLYWHEEL || state == AutoState.SHOOT_FIRST
                || state == AutoState.RETURN_TO_SHOOT || state == AutoState.WAIT_SECOND_FLYWHEEL
                || state == AutoState.SHOOT_SECOND) {
            follower.follow(line(current, hiveClearancePose).linear(current, hiveClearancePose));
            setState(AutoState.PARK_CLEAR_HIVE);
            return;
        }

        if (current.y() < flowerExitPose.y() && current.x() < flowerExitPose.x()) {
            // Route around Flower A if parking is requested from the garden.
            Pose control = new Pose(shootPose.x(), flowerExitPose.y(), current.heading());
            follower.follow(curve(current, control, parkPose).linear(current, parkPose));
        } else {
            follower.follow(line(current, parkPose).linear(current, parkPose));
        }
        setState(AutoState.PARK);
    }

    private double remainingAutoMs() {

        return AUTONOMOUS_LENGTH_MS
                - getRuntime() * 1000.0;
    }

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

    @Override
    public void stop() {
        if (follower != null) {
            OpModeStorage.autonomousEndPose = follower.pose();
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
        if (intake_servo != null && (!isFlowerState() || flowerIsClear())) {
            intake_servo.setPosition(INTAKE_SERVO_IN);
        }
        if (shooter_servo != null && shooter_servo2 != null) {
            closeShooter();
        }
    }
}
