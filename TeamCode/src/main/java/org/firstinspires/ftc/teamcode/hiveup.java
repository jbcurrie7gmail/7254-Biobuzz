package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

final class hiveup implements HiveUpAutonomous.Route {
    private static final class Settings {
        long maxResultAgeMs;
        double autonomousLengthMs;
        double tagWaitReserveMs;
        double parkReserveMs;
        double flywheelVelocity;
        double flywheelP;
        double flywheelI;
        double flywheelD;
        double flywheelF;
        double flywheelWaitMs;
        double velocityTolerance;
        double velocityStableMs;
        double shootDurationMs;
        double shootServoDelayMs;
        double intakePower;
        double transferPickupPower;
        double transferShootPower;
        double cornerPickupSpeed;
        double pickupDwellMs;
        double rotateTimeoutMs;
        double approachTimeoutMs;
        double cornerPickupTimeoutMs;
        double returnTimeoutMs;
        double parkTimeoutMs;
        int flowerPickupCycles;
        double flowerSweepSpeed;
        double flowerExitSpeed;
        double flowerApproachTimeoutMs;
        double flowerSweepTimeoutMs;
        double flowerExitTimeoutMs;
        double flowerPickupBudgetMs;
        double intakeServoIn;
        double intakeServoOut;
        double shooterServoClosed;
        double shooterServoOpen;
        double shooterServo2Closed;
        double shooterServo2Open;
        Pose shootPose;
        Pose rotatePose;
        Pose pickupApproachPose;
        Pose finalCornerPose;
        Pose returnControlPose;
        Pose flowerStartPose;
        Pose flowerInPose;
        Pose flowerOutPose;
        Pose flowerExitPose;
        double hiveClearanceTolerance;
        Pose hiveClearancePose;
        Vector2D parkOne;
        Vector2D parkTwo;
        double flowerExitMargin;
        double flowerPositionTolerance;
        double shotPositionTolerance;
        double shotHeadingTolerance;
        double rotationTolerance;
        double parkFlowerClearanceControlX;
    }

    private static Settings createBlueSettings() {
        Settings settings = new Settings();
        settings.maxResultAgeMs = HiveUpAllianceDetector.MAX_RESULT_AGE_MS;
        settings.autonomousLengthMs = 30_000;
        settings.tagWaitReserveMs = 6_000;
        settings.parkReserveMs = 3_000;
        settings.flywheelVelocity = 1620;
        settings.flywheelP = 270;
        settings.flywheelI = 0;
        settings.flywheelD = 0;
        settings.flywheelF = 12.3;
        settings.flywheelWaitMs = 3000;
        settings.velocityTolerance = 20;
        settings.velocityStableMs = 200;
        settings.shootDurationMs = 1500;
        settings.shootServoDelayMs = 150;
        settings.intakePower = 1.0;
        settings.transferPickupPower = 0.5;
        settings.transferShootPower = 0.60;
        settings.cornerPickupSpeed = 15.0;
        settings.pickupDwellMs = 500;
        settings.rotateTimeoutMs = 3000;
        settings.approachTimeoutMs = 4000;
        settings.cornerPickupTimeoutMs = 4000;
        settings.returnTimeoutMs = 5000;
        settings.parkTimeoutMs = 8000;
        settings.flowerPickupCycles = 4;
        settings.flowerSweepSpeed = 15.0;
        settings.flowerExitSpeed = 20.0;
        settings.flowerApproachTimeoutMs = 5000;
        settings.flowerSweepTimeoutMs = 2000;
        settings.flowerExitTimeoutMs = 2000;
        settings.flowerPickupBudgetMs = settings.flowerApproachTimeoutMs + settings.flowerSweepTimeoutMs + settings.flowerExitTimeoutMs;
        settings.intakeServoIn = 0.18;
        settings.intakeServoOut = 0.69;
        settings.shooterServoClosed = 0.88;
        settings.shooterServoOpen = 0.5;
        settings.shooterServo2Closed = 0.5;
        settings.shooterServo2Open = 0.88;
        settings.shootPose = new Pose(56, 7.75, Math.toRadians(270));
        settings.rotatePose = new Pose(56, 7.75, Math.toRadians(180));
        settings.pickupApproachPose = new Pose(24, 7.75, Math.toRadians(185));
        settings.finalCornerPose = new Pose(8, 7.75, Math.toRadians(185));
        settings.returnControlPose = new Pose(28, 11.75, Math.toRadians(270));
        settings.flowerStartPose = new Pose(19, 47, Math.toRadians(180));
        settings.flowerInPose = new Pose(12.75, 47, Math.toRadians(180));
        settings.flowerOutPose = new Pose(19, 47, Math.toRadians(180));
        settings.flowerExitPose = new Pose(22, 47, Math.toRadians(180));
        settings.hiveClearanceTolerance = 2.0;
        settings.hiveClearancePose = new Pose(24, 24, Math.toRadians(270));
        settings.parkOne = Vector2D.cartesian(12, 112);
        settings.parkTwo = Vector2D.cartesian(12, 93);
        settings.flowerExitMargin = 1;
        settings.flowerPositionTolerance = 2;
        settings.shotPositionTolerance = 2;
        settings.shotHeadingTolerance = Math.toRadians(5);
        settings.rotationTolerance = Math.toRadians(5);
        settings.parkFlowerClearanceControlX = 36;
        return settings;
    }

    private static Settings createRedSettings() {
        Settings settings = new Settings();
        settings.maxResultAgeMs = 200;
        settings.autonomousLengthMs = 30_000;
        settings.tagWaitReserveMs = 6_000;
        settings.parkReserveMs = 3_000;
        settings.flywheelVelocity = 1620;
        settings.flywheelP = 270;
        settings.flywheelI = 0;
        settings.flywheelD = 0;
        settings.flywheelF = 12.3;
        settings.flywheelWaitMs = 3000;
        settings.velocityTolerance = 20;
        settings.velocityStableMs = 200;
        settings.shootDurationMs = 1500;
        settings.shootServoDelayMs = 150;
        settings.intakePower = 1.0;
        settings.transferPickupPower = 0.5;
        settings.transferShootPower = 0.60;
        settings.cornerPickupSpeed = 15.0;
        settings.pickupDwellMs = 500;
        settings.rotateTimeoutMs = 3000;
        settings.approachTimeoutMs = 4000;
        settings.cornerPickupTimeoutMs = 4000;
        settings.returnTimeoutMs = 5000;
        settings.parkTimeoutMs = 8000;
        settings.flowerPickupCycles = 4;
        settings.flowerSweepSpeed = 15.0;
        settings.flowerExitSpeed = 20.0;
        settings.flowerApproachTimeoutMs = 5000;
        settings.flowerSweepTimeoutMs = 2000;
        settings.flowerExitTimeoutMs = 2000;
        settings.flowerPickupBudgetMs = settings.flowerApproachTimeoutMs + settings.flowerSweepTimeoutMs + settings.flowerExitTimeoutMs;
        settings.intakeServoIn = 0.18;
        settings.intakeServoOut = 0.69;
        settings.shooterServoClosed = 0.88;
        settings.shooterServoOpen = 0.5;
        settings.shooterServo2Closed = 0.5;
        settings.shooterServo2Open = 0.88;
        settings.shootPose = new Pose(56, 7.75, Math.toRadians(270));
        settings.rotatePose = new Pose(56, 7.75, Math.toRadians(180));
        settings.pickupApproachPose = new Pose(24, 7.75, Math.toRadians(185));
        settings.finalCornerPose = new Pose(8, 7.75, Math.toRadians(185));
        settings.returnControlPose = new Pose(28, 11.75, Math.toRadians(270));
        settings.flowerStartPose = new Pose(19, 47, Math.toRadians(180));
        settings.flowerInPose = new Pose(12.75, 47, Math.toRadians(180));
        settings.flowerOutPose = new Pose(19, 47, Math.toRadians(180));
        settings.flowerExitPose = new Pose(22, 47, Math.toRadians(180));
        settings.hiveClearanceTolerance = 2.0;
        settings.hiveClearancePose = new Pose(24, 24, Math.toRadians(270));
        settings.parkOne = Vector2D.cartesian(12, 112);
        settings.parkTwo = Vector2D.cartesian(12, 93);
        settings.flowerExitMargin = 1;
        settings.flowerPositionTolerance = 2;
        settings.shotPositionTolerance = 2;
        settings.shotHeadingTolerance = Math.toRadians(5);
        settings.rotationTolerance = Math.toRadians(5);
        settings.parkFlowerClearanceControlX = 36;
        return settings;
    }

    private final Settings blueSettings = createBlueSettings();
    private final Settings redSettings = createRedSettings();
    private Settings settings;
    private Follower follower;
    private AutonParking parking;
    private Telemetry telemetry;
    private final Timer runtime = new Timer();
    private Limelight3A limelight;
    private HiveUpAllianceDetector allianceDetector;
    private OpModeStorage.Alliance alliance;
    private HiveUpRouteChecks.FlowerExit flowerExit;
    private boolean flowerArmDown;

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
    public void init(HardwareMap hardwareMap, Telemetry telemetry, AutonParking parking) {
        this.telemetry = telemetry;
        this.parking = parking;

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

        intake_motor.setPower(0);
        transfer_motor.setPower(0);
        flywheel_motor.setPower(0);

        follower = Constants.create(hardwareMap);
        applySettings(blueSettings);

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.start();
        allianceDetector = new HiveUpAllianceDetector(limelight);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Intake", "UP / STORED");
        telemetry.addData("Start X", settings.shootPose.x());
        telemetry.addData("Start Y", settings.shootPose.y());
        parking.show(telemetry, settings.parkOne, settings.parkTwo);
        telemetry.update();
    }

    private void applySettings(Settings selected) {
        if (settings == selected) {
            return;
        }
        settings = selected;
        flowerExit = new HiveUpRouteChecks.FlowerExit(
                settings.flowerInPose, settings.flowerExitPose, settings.flowerExitMargin);
        flywheel_motor.setVelocityPIDFCoefficients(
                settings.flywheelP,
                settings.flywheelI,
                settings.flywheelD,
                settings.flywheelF
        );
        intake_servo.setPosition(settings.intakeServoIn);
        closeShooter();
        follower.setPose(settings.shootPose);
    }

    @Override
    public void initLoop(OpModeStorage.Alliance allianceOverride) {
        allianceDetector.update(allianceOverride, telemetry);
        applySettings(allianceDetector.selectedAlliance() == OpModeStorage.Alliance.RED
                ? redSettings : blueSettings);
        telemetry.addData("Configured field start (in)", "X %.2f, Y %.2f",
                settings.shootPose.x(), settings.shootPose.y());
        parking.show(telemetry, settings.parkOne, settings.parkTwo);

        telemetry.update();
    }

    @Override
    public void start() {
        alliance = allianceDetector.finish();
        applySettings(alliance == OpModeStorage.Alliance.RED ? redSettings : blueSettings);
        OpModeStorage.beginAutonomous(alliance);
        timeoutCount = 0;
        lastTimeout = "None";
        secondTagId = -1;
        flowerCyclesCompleted = 0;
        runtime.reset();

        intake_servo.setPosition(settings.intakeServoIn);
        closeShooter();
        stopFeeding();
        flywheel_motor.setVelocity(settings.flywheelVelocity);
        setState(AutoState.WAIT_FIRST_FLYWHEEL);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("Alliance / pipeline", "%s / %d (%s)", alliance,
                alliance.aprilTagPipeline, allianceDetector.selectionSource());

        double remainingMs = remainingAutoMs();
        if (remainingMs <= settings.parkReserveMs
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
            flywheel_motor.setVelocity(settings.flywheelVelocity);
        }

        switch (state) {
            case WAIT_FIRST_FLYWHEEL:

                stopFeeding();
                closeShooter();

                if (flywheelReady()) {

                    beginShooting(AutoState.SHOOT_FIRST);

                } else if (stateTimer.milliseconds()
                        >= settings.flywheelWaitMs) {

                    recordTimeout();
                    beginShooting(AutoState.SHOOT_FIRST);
                }
                break;

            case SHOOT_FIRST:

                runShooting();

                if (stateTimer.milliseconds()
                        >= settings.shootDurationMs) {

                    stopFeeding();
                    closeShooter();

                    follower.hold(settings.rotatePose);
                    setState(AutoState.ROTATE_TO_PICKUP);
                }
                break;

            case ROTATE_TO_PICKUP:

                double headingError = Math.atan2(
                        Math.sin(
                                settings.rotatePose.heading()
                                        - follower.pose().heading()
                        ),
                        Math.cos(
                                settings.rotatePose.heading()
                                        - follower.pose().heading()
                        )
                );

                boolean rotationComplete =
                        Math.abs(headingError)
                                < settings.rotationTolerance;

                boolean rotationTimedOut =
                        stateTimer.milliseconds()
                                >= settings.rotateTimeoutMs;

                if (rotationComplete || rotationTimedOut) {

                    if (rotationTimedOut && !rotationComplete) {
                        recordTimeout();
                        follower.stop();
                    }

                    startIntake();

                    followLine(settings.pickupApproachPose);

                    setState(AutoState.DRIVE_TO_PICKUP);
                }
                break;

            case DRIVE_TO_PICKUP:

                startIntake();

                if (pathFinishedOrTimedOut(
                        settings.approachTimeoutMs)) {

                    followLine(
                            settings.finalCornerPose,
                            settings.cornerPickupSpeed
                    );

                    setState(AutoState.CORNER_PICKUP);
                }
                break;

            case CORNER_PICKUP:

                startIntake();

                if (pathFinishedOrTimedOut(
                        settings.cornerPickupTimeoutMs)) {

                    setState(AutoState.FINISH_PICKUP);
                }
                break;

            case FINISH_PICKUP:

                startIntake();

                if (stateTimer.milliseconds()
                        >= settings.pickupDwellMs) {

                    stopFeeding();

                    Pose current = follower.pose();

                    Path returnPath =
                            curve(
                                    current,
                                    settings.returnControlPose,
                                    settings.shootPose
                            ).linear(
                                    current,
                                    settings.shootPose
                            );

                    follower.follow(returnPath);

                    setState(AutoState.RETURN_TO_SHOOT);
                }
                break;

            case RETURN_TO_SHOOT:
                stopFeeding();
                closeShooter();

                if (pathFinished() && HiveUpRouteChecks.atShot(follower.pose(), settings.shootPose,
                        settings.shotPositionTolerance, settings.shotHeadingTolerance)) {
                    // Retain the readiness measured during the return drive.
                    state = AutoState.WAIT_SECOND_FLYWHEEL;
                    stateTimer.reset();
                    checkForSecondShot(remainingMs);
                } else if (stateTimer.milliseconds() >= settings.returnTimeoutMs) {
                    recordTimeout();
                    beginPark();
                } else {
                    flywheelReady();
                }
                break;

            case WAIT_SECOND_FLYWHEEL:
                checkForSecondShot(remainingMs);
                break;

            case SHOOT_SECOND:
                runShooting();
                if (stateTimer.milliseconds() >= settings.shootDurationMs) {
                    if (remainingMs >= settings.flowerPickupBudgetMs + settings.parkReserveMs) {
                        beginFlowerApproach();
                    } else {
                        beginPark();
                    }
                }
                break;

            case DRIVE_TO_FLOWER:
                if (!hasTimeForFlowerSweep(remainingMs)) {
                    beginFlowerExit();
                } else if (pathFinished() && nearFlowerPose(settings.flowerStartPose)) {
                    beginFlowerSweepIn();
                } else if (stateTimer.milliseconds() >= settings.flowerApproachTimeoutMs) {
                    recordTimeout();
                    beginFlowerExit();
                }
                break;

            case FLOWER_SWEEP_IN:
                if (remainingMs <= settings.parkReserveMs + settings.flowerExitTimeoutMs) {
                    beginFlowerExit();
                } else if (pathFinished() && nearFlowerPose(settings.flowerInPose)) {
                    beginFlowerSweepOut();
                } else if (stateTimer.milliseconds() >= settings.flowerSweepTimeoutMs) {
                    recordTimeout();
                    beginFlowerExit();
                }
                break;

            case FLOWER_SWEEP_OUT:
                if (pathFinished() && nearFlowerPose(settings.flowerOutPose)) {
                    flowerCyclesCompleted++;
                    if (flowerCyclesCompleted < settings.flowerPickupCycles
                            && hasTimeForFlowerSweep(remainingMs)) {
                        beginFlowerSweepIn();
                    } else {
                        beginFlowerExit();
                    }
                } else if (remainingMs <= settings.parkReserveMs + settings.flowerExitTimeoutMs) {
                    beginFlowerExit();
                } else if (stateTimer.milliseconds() >= settings.flowerSweepTimeoutMs) {
                    recordTimeout();
                    beginFlowerExit();
                }
                break;

            case FLOWER_EXIT:
                if (flowerIsClear()) {
                    beginPark();
                } else if (remainingMs <= 0) {
                    follower.stop();
                    stopFeeding();
                    closeShooter();
                    setState(AutoState.DONE);
                } else if (stateTimer.milliseconds() >= settings.flowerExitTimeoutMs) {
                    recordTimeout();
                    beginFlowerExit();
                }
                break;

            case PARK_CLEAR_HIVE:
                Pose current = follower.pose();
                if (pathFinished()
                        && Math.hypot(current.x() - settings.hiveClearancePose.x(),
                                current.y() - settings.hiveClearancePose.y()) <= settings.hiveClearanceTolerance) {
                    // Keep the park leg on the clear side of Flower A.
                    Pose parkTarget = parking.targetFrom(current, settings.parkOne, settings.parkTwo);
                    Pose control = new Pose(settings.parkFlowerClearanceControlX, settings.flowerExitPose.y(), parkTarget.heading());
                    follower.follow(curve(current, control, parkTarget).linear(current, parkTarget));
                    setState(AutoState.PARK);
                } else if (stateTimer.milliseconds() >= settings.parkTimeoutMs) {
                    recordTimeout();
                    follower.stop();
                    setState(AutoState.DONE);
                }
                break;

            case PARK:
                if (pathFinishedOrTimedOut(settings.parkTimeoutMs)) {
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
        telemetry.addData("Flower A cycles", "%d / %d", flowerCyclesCompleted, settings.flowerPickupCycles);
        telemetry.addData("Timeouts", timeoutCount);
        telemetry.addData("Last timeout", lastTimeout);
        telemetry.update();
    }

    private void checkForSecondShot(double remainingMs) {
        if (secondTagId < 0) {
            if (remainingMs <= settings.tagWaitReserveMs) {
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
            beginVerifiedSecondShot();
        } else if (stateTimer.milliseconds() >= settings.flywheelWaitMs
                || remainingMs <= settings.parkReserveMs + settings.shootDurationMs + settings.shootServoDelayMs) {
            recordTimeout();
            beginVerifiedSecondShot();
        }
    }

    private void beginVerifiedSecondShot() {
        if (HiveUpRouteChecks.atShot(follower.pose(), settings.shootPose,
                settings.shotPositionTolerance, settings.shotHeadingTolerance)) {
            beginShooting(AutoState.SHOOT_SECOND);
        } else {
            beginPark();
        }
    }

    private int visibleAprilTagId() {
        if (!limelight.isConnected()) {
            return -1;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()
                || result.getPipelineIndex() != alliance.aprilTagPipeline
                || result.getStaleness() > settings.maxResultAgeMs) {
            return -1;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        return tags == null || tags.isEmpty() ? -1 : tags.get(0).getFiducialId();
    }

    private boolean hasTimeForFlowerSweep(double remainingMs) {
        return remainingMs > settings.parkReserveMs + settings.flowerExitTimeoutMs + settings.flowerSweepTimeoutMs;
    }

    private boolean isFlowerState() {
        return state == AutoState.DRIVE_TO_FLOWER
                || state == AutoState.FLOWER_SWEEP_IN
                || state == AutoState.FLOWER_SWEEP_OUT
                || state == AutoState.FLOWER_EXIT;
    }

    private boolean pathFinished() {
        return follower.mode() != Follower.Mode.FOLLOW;
    }

    private boolean nearFlowerPose(Pose target) {
        return HiveUpRouteChecks.near(follower.pose(), target, settings.flowerPositionTolerance);
    }

    private boolean flowerIsClear() {
        return flowerExit.isClear(follower.pose());
    }

    private void beginFlowerApproach() {

        stopFeeding();
        closeShooter();
        flywheel_motor.setPower(0);

        intake_servo.setPosition(settings.intakeServoOut);
        flowerArmDown = true;
        flowerCyclesCompleted = 0;

        followLine(settings.flowerStartPose);

        setState(AutoState.DRIVE_TO_FLOWER);
    }

    private void beginFlowerSweepIn() {

        intake_servo.setPosition(settings.intakeServoOut);

        startIntake();
        closeShooter();

        followLine(
                settings.flowerInPose,
                settings.flowerSweepSpeed
        );

        setState(AutoState.FLOWER_SWEEP_IN);
    }

    private void beginFlowerSweepOut() {

        intake_servo.setPosition(settings.intakeServoOut);

        startIntake();
        closeShooter();

        followLine(
                settings.flowerOutPose,
                settings.flowerSweepSpeed
        );

        setState(AutoState.FLOWER_SWEEP_OUT);
    }

    private void beginFlowerExit() {

        intake_servo.setPosition(settings.intakeServoOut);

        startIntake();
        closeShooter();

        followLine(
                flowerExit.target,
                settings.flowerExitSpeed
        );

        setState(AutoState.FLOWER_EXIT);
    }

    private void beginPark() {
        if (flowerArmDown && !flowerIsClear()) {
            beginFlowerExit();
            return;
        }
        flowerArmDown = false;
        stopFeeding();
        closeShooter();
        flywheel_motor.setPower(0);
        intake_servo.setPosition(settings.intakeServoIn);

        Pose current = follower.pose();
        if (state == AutoState.WAIT_FIRST_FLYWHEEL || state == AutoState.SHOOT_FIRST
                || state == AutoState.RETURN_TO_SHOOT || state == AutoState.WAIT_SECOND_FLYWHEEL
                || state == AutoState.SHOOT_SECOND) {
            follower.follow(line(current, settings.hiveClearancePose).linear(current, settings.hiveClearancePose));
            setState(AutoState.PARK_CLEAR_HIVE);
            return;
        }

        Pose parkTarget = parking.targetFrom(current, settings.parkOne, settings.parkTwo);
        if (current.y() < settings.flowerExitPose.y() && current.x() < settings.flowerExitPose.x()) {
            // Route around Flower A if parking is requested from the garden.
            Pose control = new Pose(settings.shootPose.x(), settings.flowerExitPose.y(), current.heading());
            follower.follow(curve(current, control, parkTarget).linear(current, parkTarget));
        } else {
            follower.follow(line(current, parkTarget).linear(current, parkTarget));
        }
        setState(AutoState.PARK);
    }

    private double remainingAutoMs() {

        return settings.autonomousLengthMs
                - runtime.seconds() * 1000.0;
    }

    private void followLine(Pose target) {
        Pose current = follower.pose();
        follower.follow(line(current, target).linear(current, target));
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

        if (pathFinished()) {
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
                settings.flywheelVelocity - actualVelocity
        );

        if (velocityError <= settings.velocityTolerance) {

            if (!velocityStableTracking) {

                velocityTimer.reset();
                velocityStableTracking = true;
            }

            return velocityTimer.milliseconds()
                    >= settings.velocityStableMs;

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
                >= settings.shootServoDelayMs) {

            intake_motor.setPower(
                    firstShooting ? settings.intakePower : 0
            );

            transfer_motor.setPower(
                    settings.transferShootPower
            );

        } else {

            stopFeeding();
        }
    }

    private void openShooter() {

        shooter_servo.setPosition(
                settings.shooterServoOpen
        );

        shooter_servo2.setPosition(
                settings.shooterServo2Open
        );
    }

    private void closeShooter() {

        shooter_servo.setPosition(
                settings.shooterServoClosed
        );

        shooter_servo2.setPosition(
                settings.shooterServo2Closed
        );
    }

    private void startIntake() {

        intake_motor.setPower(settings.intakePower);

        transfer_motor.setPower(
                settings.transferPickupPower
        );
    }

    private void stopFeeding() {

        intake_motor.setPower(0);
        transfer_motor.setPower(0);
    }

    @Override
    public void stop(boolean saveEndPose) {
        if (follower != null) {
            if (saveEndPose) {
                Pose endPose = follower.pose();
                OpModeStorage.saveAutonomousEnd(endPose,
                        flowerArmDown && !flowerExit.isClear(endPose) ? flowerExit : null, settings.intakeServoOut);
            }
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
        if (intake_servo != null && settings != null && (!flowerArmDown || (follower != null && flowerIsClear()))) {
            intake_servo.setPosition(settings.intakeServoIn);
        }
        if (shooter_servo != null && shooter_servo2 != null && settings != null) {
            closeShooter();
        }
    }
}
