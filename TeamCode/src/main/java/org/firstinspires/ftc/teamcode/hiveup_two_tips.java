
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

final class hiveup_two_tips implements HiveUpAutonomous.Route {
    private static final class Settings {
        long maxResultAgeMs;
        double tagWaitMs;
        double flywheelVelocity;
        double alternateFlywheelVelocity;
        double secondaryFlywheelVelocity;
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
        double alternateTravelTimeoutMs;
        double alternateSettleMs;
        double autonomousLengthMs;
        double alternateParkReserveMs;
        double alternateFlowerBudgetMs;
        double alternateClearanceBudgetMs;
        int alternateFlowerPickupCycles;
        double alternateFlowerSpeed;
        double alternateClearanceTimeoutMs;
        double alternateFlowerApproachTimeoutMs;
        double alternateFlowerInTimeoutMs;
        double alternateFlowerOutTimeoutMs;
        double alternateFlowerRecoveryTimeoutMs;
        double alternateFlowerExitTolerance;
        double secondaryTravelTimeoutMs;
        double secondaryParkReserveMs;
        double secondaryFlywheelWaitMs;
        double secondaryShootDurationMs;
        int flowerPickupCycles;
        double flowerSweepSpeed;
        double flowerExitSpeed;
        double flowerApproachTimeoutMs;
        double flowerSweepTimeoutMs;
        double flowerExitTimeoutMs;
        double flowerPositionTolerance;
        double flowerExitMargin;
        double alternateFlowerExitMargin;
        double shotPositionTolerance;
        double shotHeadingTolerance;
        Pose hiveClearancePose;
        double hiveClearanceTolerance;
        double parkFlowerClearanceControlX;
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
        Pose alternateShootPose;
        Pose alternateFlowerClearancePose;
        Pose alternateFlowerStartPose;
        Pose alternateFlowerEndPose;
        Pose secondaryShootPose;
        Pose flowerStartPose;
        Pose flowerInPose;
        Pose flowerOutPose;
        Pose flowerExitPose;
        Vector2D parkOne;
        Vector2D parkTwo;
        double rotationTolerance;
    }

    private static Settings createBlueSettings() {
        Settings settings = new Settings();
        settings.maxResultAgeMs = HiveUpAllianceDetector.MAX_RESULT_AGE_MS;
        settings.tagWaitMs = 500;
        settings.flywheelVelocity = 1620;
        settings.alternateFlywheelVelocity = 1620;
        settings.secondaryFlywheelVelocity = 1600;
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
        settings.alternateTravelTimeoutMs = 10000;
        settings.alternateSettleMs = 1000;
        settings.autonomousLengthMs = 30000;
        settings.alternateParkReserveMs = 3000;
        settings.alternateFlowerBudgetMs = 4500;
        settings.alternateClearanceBudgetMs = 1500;
        settings.alternateFlowerPickupCycles = 3;
        settings.alternateFlowerSpeed = 20.0;
        settings.alternateClearanceTimeoutMs = 2000;
        settings.alternateFlowerApproachTimeoutMs = 2500;
        settings.alternateFlowerInTimeoutMs = 1500;
        settings.alternateFlowerOutTimeoutMs = 2000;
        settings.alternateFlowerRecoveryTimeoutMs = 1800;
        settings.alternateFlowerExitTolerance = 2.0;
        settings.secondaryTravelTimeoutMs = 1000;
        settings.secondaryParkReserveMs = 1000;
        settings.secondaryFlywheelWaitMs = 750;
        settings.secondaryShootDurationMs = 1000;
        settings.flowerPickupCycles = 4;
        settings.flowerSweepSpeed = 15.0;
        settings.flowerExitSpeed = 20.0;
        settings.flowerApproachTimeoutMs = 5000;
        settings.flowerSweepTimeoutMs = 2000;
        settings.flowerExitTimeoutMs = 2000;
        settings.flowerPositionTolerance = 2;
        settings.flowerExitMargin = 1;
        settings.alternateFlowerExitMargin = 1;
        settings.shotPositionTolerance = 2;
        settings.shotHeadingTolerance = Math.toRadians(5);
        settings.hiveClearancePose = new Pose(24, 24, Math.toRadians(270));
        settings.hiveClearanceTolerance = 2;
        settings.parkFlowerClearanceControlX = 36;
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
        settings.alternateShootPose = new Pose(59, 132, Math.toRadians(88));
        settings.alternateFlowerClearancePose = new Pose(59, 117, Math.toRadians(88));
        settings.alternateFlowerStartPose = new Pose(48, 122, Math.toRadians(90));
        settings.alternateFlowerEndPose = new Pose(48, 126.5, Math.toRadians(90));
        settings.secondaryShootPose = new Pose(33.0, 122.0, Math.toRadians(120));
        settings.flowerStartPose = new Pose(19, 47, Math.toRadians(180));
        settings.flowerInPose = new Pose(12.75, 47, Math.toRadians(180));
        settings.flowerOutPose = new Pose(19, 47, Math.toRadians(180));
        settings.flowerExitPose = new Pose(22, 47, Math.toRadians(180));
        settings.parkOne = Vector2D.cartesian(12, 112);
        settings.parkTwo = Vector2D.cartesian(12, 93);
        settings.rotationTolerance = Math.toRadians(5);
        return settings;
    }

    private static Settings createRedSettings() {
        Settings settings = new Settings();
        settings.maxResultAgeMs = 200;
        settings.tagWaitMs = 500;
        settings.flywheelVelocity = 1620;
        settings.alternateFlywheelVelocity = 1620;
        settings.secondaryFlywheelVelocity = 1600;
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
        settings.alternateTravelTimeoutMs = 10000;
        settings.alternateSettleMs = 1000;
        settings.autonomousLengthMs = 30000;
        settings.alternateParkReserveMs = 3000;
        settings.alternateFlowerBudgetMs = 4500;
        settings.alternateClearanceBudgetMs = 1500;
        settings.alternateFlowerPickupCycles = 3;
        settings.alternateFlowerSpeed = 20.0;
        settings.alternateClearanceTimeoutMs = 2000;
        settings.alternateFlowerApproachTimeoutMs = 2500;
        settings.alternateFlowerInTimeoutMs = 1500;
        settings.alternateFlowerOutTimeoutMs = 2000;
        settings.alternateFlowerRecoveryTimeoutMs = 1800;
        settings.alternateFlowerExitTolerance = 2.0;
        settings.secondaryTravelTimeoutMs = 1000;
        settings.secondaryParkReserveMs = 1000;
        settings.secondaryFlywheelWaitMs = 750;
        settings.secondaryShootDurationMs = 1000;
        settings.flowerPickupCycles = 4;
        settings.flowerSweepSpeed = 15.0;
        settings.flowerExitSpeed = 20.0;
        settings.flowerApproachTimeoutMs = 5000;
        settings.flowerSweepTimeoutMs = 2000;
        settings.flowerExitTimeoutMs = 2000;
        settings.flowerPositionTolerance = 2;
        settings.flowerExitMargin = 1;
        settings.alternateFlowerExitMargin = 1;
        settings.shotPositionTolerance = 2;
        settings.shotHeadingTolerance = Math.toRadians(5);
        settings.hiveClearancePose = new Pose(24, 24, Math.toRadians(270));
        settings.hiveClearanceTolerance = 2;
        settings.parkFlowerClearanceControlX = 36;
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
        settings.alternateShootPose = new Pose(59, 132, Math.toRadians(88));
        settings.alternateFlowerClearancePose = new Pose(59, 117, Math.toRadians(88));
        settings.alternateFlowerStartPose = new Pose(48, 122, Math.toRadians(90));
        settings.alternateFlowerEndPose = new Pose(48, 126.5, Math.toRadians(90));
        settings.secondaryShootPose = new Pose(33.0, 122.0, Math.toRadians(120));
        settings.flowerStartPose = new Pose(19, 47, Math.toRadians(180));
        settings.flowerInPose = new Pose(12.75, 47, Math.toRadians(180));
        settings.flowerOutPose = new Pose(19, 47, Math.toRadians(180));
        settings.flowerExitPose = new Pose(22, 47, Math.toRadians(180));
        settings.parkOne = Vector2D.cartesian(12, 112);
        settings.parkTwo = Vector2D.cartesian(12, 93);
        settings.rotationTolerance = Math.toRadians(5);
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
    private HiveUpRouteChecks.FlowerExit primaryFlowerExit;
    private HiveUpRouteChecks.FlowerExit alternateFlowerExit;
    private HiveUpRouteChecks.FlowerExit activeFlowerExit;

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

        PARK_CLEAR_HIVE,
        PARK,
        DONE
    }

    private AutoState state =
            AutoState.WAIT_FIRST_FLYWHEEL;

    // =========================================
    // INITIALIZATION
    // =========================================

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
        primaryFlowerExit = new HiveUpRouteChecks.FlowerExit(
                settings.flowerInPose, settings.flowerExitPose, settings.flowerExitMargin);
        alternateFlowerExit = new HiveUpRouteChecks.FlowerExit(
                settings.alternateFlowerEndPose, settings.alternateFlowerStartPose,
                settings.alternateFlowerExitMargin);
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

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        alliance = allianceDetector.finish();
        applySettings(alliance == OpModeStorage.Alliance.RED ? redSettings : blueSettings);
        OpModeStorage.beginAutonomous(alliance);

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

        runtime.reset();

        intake_servo.setPosition(settings.intakeServoIn);
        closeShooter();
        stopFeeding();

        flywheel_motor.setVelocity(settings.flywheelVelocity);

        setState(AutoState.WAIT_FIRST_FLYWHEEL);
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        follower.update();
        telemetry.addData("Alliance / pipeline", "%s / %d (%s)", alliance,
                alliance.aprilTagPipeline, allianceDetector.selectionSource());

        // UPDATED:
        // Flywheel velocity selected by state.
        if (isFlywheelEnabled()) {
            flywheel_motor.setVelocity(flywheelTargetVelocity());
        } else {
            flywheel_motor.setPower(0);
        }

        double remainingMs = remainingAutoMs();
        prioritizeParking(remainingMs);

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

            // =================================
            // ROTATE AND CORNER PICKUP
            // =================================

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

            // =================================
            // PRIMARY SECOND SHOT
            // =================================

            case RETURN_TO_SHOOT:
                stopFeeding();
                closeShooter();
                if (pathFinished() && atShot(settings.shootPose)) {
                    secondTagSeen = false;
                    secondTagId = -1;
                    secondShotStatus = "Searching for AprilTag";
                    arrivalFrameTimestamp = latestFrameTimestamp();
                    setState(AutoState.WAIT_SECOND_FLYWHEEL);
                } else if (stateTimer.milliseconds() >= settings.returnTimeoutMs) {
                    recordTimeout();
                    secondShotStatus = "Primary return timeout - parking";
                    beginPark();
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
                            >= settings.tagWaitMs) {

                        secondShotStatus =
                                "No primary tag - alternate side";

                        beginAlternateTravel();
                    }

                } else if (ready) {

                    secondShotStatus = "Primary second shot";
                    beginVerifiedShot(AutoState.SHOOT_SECOND, settings.shootPose);

                } else if (stateTimer.milliseconds()
                        >= settings.flywheelWaitMs) {

                    recordTimeout();

                    secondShotStatus =
                            "Primary shot - flywheel timeout";

                    beginVerifiedShot(AutoState.SHOOT_SECOND, settings.shootPose);
                }
                break;

            case SHOOT_SECOND:
                runShooting();
                if (stateTimer.milliseconds() >= settings.shootDurationMs) {
                    double flowerBudget = settings.flowerApproachTimeoutMs
                            + settings.flowerSweepTimeoutMs + settings.flowerExitTimeoutMs;
                    if (remainingMs >= settings.alternateParkReserveMs + flowerBudget) {
                        beginFlowerApproach();
                    } else {
                        beginPark();
                    }
                }
                break;

            case DRIVE_TO_FLOWER:
                if (pathFinished() && near(settings.flowerStartPose, settings.flowerPositionTolerance)) {
                    beginFlowerSweepIn();
                } else if (stateTimer.milliseconds() >= settings.flowerApproachTimeoutMs) {
                    recordTimeout();
                    beginPark();
                }
                break;

            case FLOWER_SWEEP_IN:
                if (pathFinished() && near(settings.flowerInPose, settings.flowerPositionTolerance)) {
                    beginFlowerSweepOut();
                } else if (stateTimer.milliseconds() >= settings.flowerSweepTimeoutMs) {
                    recordTimeout();
                    beginPark();
                }
                break;

            case FLOWER_SWEEP_OUT:
                if (pathFinished() && near(settings.flowerOutPose, settings.flowerPositionTolerance)) {
                    flowerCyclesCompleted++;
                    double nextSweepBudget = settings.flowerSweepTimeoutMs + settings.flowerExitTimeoutMs;
                    if (flowerCyclesCompleted < settings.flowerPickupCycles
                            && remainingMs > settings.alternateParkReserveMs + nextSweepBudget) {
                        beginFlowerSweepIn();
                    } else {
                        beginFlowerExit();
                    }
                } else if (stateTimer.milliseconds() >= settings.flowerSweepTimeoutMs) {
                    recordTimeout();
                    beginPark();
                }
                break;

            case FLOWER_EXIT:
                if (primaryFlowerExit.isClear(follower.pose())) {
                    beginPark();
                } else if (remainingMs <= 0) {
                    stopInsideFlower();
                } else if (stateTimer.milliseconds() >= settings.flowerExitTimeoutMs) {
                    recordTimeout();
                    beginFlowerExit();
                }
                break;

            case DRIVE_TO_ALTERNATE_SHOT:
                if (pathFinished() && atShot(settings.alternateShootPose)) {
                    alternateTagSeen = false;
                    alternateTagId = -1;
                    alternateArrivalFrameTimestamp = latestFrameTimestamp();
                    secondShotStatus = "Checking alternate AprilTag";
                    setState(AutoState.WAIT_ALTERNATE_TAG);
                } else if (stateTimer.milliseconds() >= settings.alternateTravelTimeoutMs) {
                    recordTimeout();
                    secondShotStatus = "Alternate travel timeout";
                    beginPark();
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
                        >= settings.alternateSettleMs) {

                    if (alternateTagSeen) {

                        beginVerifiedShot(AutoState.SHOOT_ALTERNATE, settings.alternateShootPose);

                    } else {

                        secondShotStatus =
                                "No alternate tag - parking";

                        beginPark();
                    }
                }
                break;

            case SHOOT_ALTERNATE:

                runShooting();

                if (stateTimer.milliseconds()
                        >= settings.shootDurationMs) {

                    secondShotStatus =
                            "First alternate shot complete";

                    if (remainingAutoMs()
                            >= settings.alternateParkReserveMs
                            + settings.alternateFlowerBudgetMs
                            + settings.alternateClearanceBudgetMs) {

                        beginAlternateFlowerClearance();

                    } else {

                        secondShotStatus =
                                "Not enough time for flower - park";

                        beginPark();
                    }
                }
                break;

            // =================================
            // ALTERNATE FLOWER CLEARANCE
            // =================================

            case ALT_FLOWER_CLEARANCE:
                if (pathFinished() && near(settings.alternateFlowerClearancePose,
                        settings.alternateFlowerExitTolerance)) {
                    beginAlternateFlowerApproach();
                } else if (stateTimer.milliseconds() >= settings.alternateClearanceTimeoutMs) {
                    recordTimeout();
                    secondShotStatus = "Clearance timeout - parking";
                    beginPark();
                }
                break;

            case ALT_FLOWER_APPROACH:
                if (pathFinished() && near(settings.alternateFlowerStartPose,
                        settings.alternateFlowerExitTolerance)) {
                    beginAlternateFlowerIn();
                } else if (stateTimer.milliseconds() >= settings.alternateFlowerApproachTimeoutMs) {
                    recordTimeout();
                    beginPark();
                }
                break;

            case ALT_FLOWER_PICKUP_IN:
                if (pathFinished() && near(settings.alternateFlowerEndPose,
                        settings.alternateFlowerExitTolerance)) {
                    beginAlternateFlowerOut();
                } else if (stateTimer.milliseconds() >= settings.alternateFlowerInTimeoutMs) {
                    recordTimeout();
                    secondShotStatus = "Flower IN timeout - recovering exit";
                    beginPark();
                }
                break;

            case ALT_FLOWER_PICKUP_OUT:
                if (pathFinished() && alternateFlowerExit.isClear(follower.pose())) {
                    alternateFlowerCyclesCompleted++;
                    double nextSweepBudget = settings.alternateFlowerInTimeoutMs
                            + settings.alternateFlowerRecoveryTimeoutMs;
                    if (alternateFlowerCyclesCompleted >= settings.alternateFlowerPickupCycles) {
                        finishAlternateFlowerPickup(remainingMs);
                    } else if (remainingMs > settings.alternateParkReserveMs + nextSweepBudget) {
                        beginAlternateFlowerIn();
                    } else {
                        beginPark();
                    }
                } else if (stateTimer.milliseconds() >= settings.alternateFlowerOutTimeoutMs) {
                    recordTimeout();
                    beginPark();
                }
                break;

            case ALT_FLOWER_RECOVERY:
                if (alternateFlowerExit.isClear(follower.pose())) {
                    beginPark();
                } else if (remainingMs <= 0) {
                    stopInsideFlower();
                } else if (stateTimer.milliseconds() >= settings.alternateFlowerRecoveryTimeoutMs) {
                    recordTimeout();
                    beginAlternateFlowerRecovery();
                }
                break;

            case ALT_DRIVE_TO_SECONDARY_SHOT:
                if (pathFinished() && atShot(settings.secondaryShootPose)) {
                    secondShotStatus = "At secondary shoot position";
                    setState(AutoState.ALT_WAIT_SECONDARY_FLYWHEEL);
                } else if (stateTimer.milliseconds() >= settings.secondaryTravelTimeoutMs) {
                    recordTimeout();
                    secondShotStatus = "Secondary shooting travel timeout";
                    beginPark();
                }
                break;

            case ALT_WAIT_SECONDARY_FLYWHEEL:

                stopFeeding();
                closeShooter();

                intake_servo.setPosition(settings.intakeServoIn);

                // flywheelReady() now checks 1600
                // specifically for secondary shooting.
                if (flywheelReady()
                        || stateTimer.milliseconds()
                        >= settings.secondaryFlywheelWaitMs) {

                    secondShotStatus =
                            "Shooting secondary position";

                    beginVerifiedShot(AutoState.ALT_SHOOT_SECONDARY, settings.secondaryShootPose);
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
                        >= settings.secondaryShootDurationMs) {

                    secondShotStatus =
                            "Secondary shot complete";

                    beginPark();
                }
                break;

            // =================================
            // PARKING
            // =================================

            case PARK_CLEAR_HIVE:
                Pose current = follower.pose();
                if (pathFinished() && near(settings.hiveClearancePose, settings.hiveClearanceTolerance)) {
                    Pose parkTarget = parking.targetFrom(current, settings.parkOne, settings.parkTwo);
                    Pose control = new Pose(settings.parkFlowerClearanceControlX,
                            settings.flowerExitPose.y(), parkTarget.heading());
                    follower.follow(curve(current, control, parkTarget)
                            .linear(current, parkTarget));
                    setState(AutoState.PARK);
                } else if (stateTimer.milliseconds() >= settings.parkTimeoutMs) {
                    recordTimeout();
                    follower.stop();
                    setState(AutoState.DONE);
                }
                break;

            case PARK:

                if (pathFinishedOrTimedOut(
                        settings.parkTimeoutMs)) {

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
                settings.flowerPickupCycles
        );

        telemetry.addData(
                "Alternate Flower Cycles Completed",
                alternateFlowerCyclesCompleted
        );

        telemetry.addData(
                "Alternate Flower Cycles Target",
                settings.alternateFlowerPickupCycles
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
                "Past Alternate Flower Exit",
                alternateFlowerExit.isClear(follower.pose())
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
                settings.alternateParkReserveMs
                        + settings.alternateFlowerBudgetMs
                        + settings.alternateClearanceBudgetMs
        );

        telemetry.addData("Timeout Count", timeoutCount);
        telemetry.addData("Last Timeout", lastTimeout);

        telemetry.update();
    }

    // =========================================
    // PATH HELPER
    // =========================================

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
            return settings.secondaryFlywheelVelocity;
        }

        return isAlternateFlywheelState()
                ? settings.alternateFlywheelVelocity
                : settings.flywheelVelocity;
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
                && state != AutoState.PARK_CLEAR_HIVE
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
                != alliance.aprilTagPipeline) return false;

        if (result.getStaleness()
                > settings.maxResultAgeMs) return false;

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
                != alliance.aprilTagPipeline) return false;

        if (result.getStaleness()
                > settings.maxResultAgeMs) return false;

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

        activeFlowerExit = primaryFlowerExit;
        intake_servo.setPosition(settings.intakeServoOut);
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
                primaryFlowerExit.target,
                settings.flowerExitSpeed
        );

        setState(AutoState.FLOWER_EXIT);
    }

    // =========================================
    // TIME MANAGEMENT
    // =========================================

    private double remainingAutoMs() {

        return settings.autonomousLengthMs
                - runtime.seconds() * 1000.0;
    }

    // =========================================
    // ALTERNATE FLOWER POSITION CHECK
    // =========================================

    // =========================================
    // ALTERNATE FLOWER HELPERS
    // =========================================

    private void beginAlternateFlowerClearance() {

        stopFeeding();
        closeShooter();
        flywheel_motor.setPower(0);

        // Arm UP during retreat.
        intake_servo.setPosition(settings.intakeServoIn);

        alternateFlowerCyclesCompleted = 0;
        alternateFlowerCompleted = false;

        followLine(settings.alternateFlowerClearancePose);

        secondShotStatus =
                "Backing away before arm deployment";

        setState(AutoState.ALT_FLOWER_CLEARANCE);
    }

    private void beginAlternateFlowerApproach() {

        stopFeeding();
        closeShooter();
        flywheel_motor.setPower(0);

        // Lower only after clearance.
        activeFlowerExit = alternateFlowerExit;
        intake_servo.setPosition(settings.intakeServoOut);

        followLine(settings.alternateFlowerStartPose);

        secondShotStatus =
                "Approaching alternate flower";

        setState(AutoState.ALT_FLOWER_APPROACH);
    }

    private void beginAlternateFlowerIn() {

        intake_servo.setPosition(settings.intakeServoOut);

        startIntake();
        closeShooter();
        flywheel_motor.setPower(0);

        followLine(
                settings.alternateFlowerEndPose,
                settings.alternateFlowerSpeed
        );

        secondShotStatus =
                "Alternate flower moving IN";

        setState(AutoState.ALT_FLOWER_PICKUP_IN);
    }

    private void beginAlternateFlowerOut() {

        // Keep arm DOWN throughout OUT movement.
        intake_servo.setPosition(settings.intakeServoOut);

        startIntake();
        closeShooter();
        flywheel_motor.setPower(0);

        followLine(
                alternateFlowerExit.target,
                settings.alternateFlowerSpeed
        );

        secondShotStatus =
                "Alternate flower moving OUT";

        setState(AutoState.ALT_FLOWER_PICKUP_OUT);
    }

    private void beginAlternateFlowerRecovery() {

        // Do not raise arm during recovery.
        intake_servo.setPosition(settings.intakeServoOut);

        startIntake();
        closeShooter();
        flywheel_motor.setPower(0);

        followLine(
                alternateFlowerExit.target,
                settings.alternateFlowerSpeed
        );

        secondShotStatus =
                "Recovering alternate flower exit";

        setState(AutoState.ALT_FLOWER_RECOVERY);
    }

    // =========================================
    // FINISH THREE ALTERNATE FLOWER CYCLES
    // =========================================

    private void finishAlternateFlowerPickup(double remainingMs) {

        // Three IN/OUT cycles completed.
        // Robot is back at (48, 122, 90).
        // NOW raise the intake.

        stopFeeding();
        closeShooter();

        intake_servo.setPosition(settings.intakeServoIn);

        activeFlowerExit = null;
        alternateFlowerCompleted = true;
        if (remainingMs >= secondaryShotBudgetMs()) {
            beginSecondaryShotTravel();
        } else {
            beginPark();
        }
    }

    // =========================================
    // SECONDARY SHOT TRAVEL
    // =========================================

    private void beginSecondaryShotTravel() {

        stopFeeding();
        closeShooter();

        intake_servo.setPosition(settings.intakeServoIn);

        followLine(settings.secondaryShootPose);

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

        intake_servo.setPosition(settings.intakeServoIn);

        followLine(settings.alternateShootPose);

        secondShotStatus =
                "Traveling to alternate shooting position";

        setState(AutoState.DRIVE_TO_ALTERNATE_SHOT);
    }

    // =========================================
    // PARK
    // =========================================

    private double secondaryShotBudgetMs() {
        return settings.secondaryTravelTimeoutMs + settings.secondaryFlywheelWaitMs
                + settings.secondaryShootDurationMs + settings.secondaryParkReserveMs;
    }

    private void prioritizeParking(double remainingMs) {
        if (state == AutoState.PARK_CLEAR_HIVE || state == AutoState.PARK
                || state == AutoState.DONE || state == AutoState.FLOWER_EXIT
                || state == AutoState.ALT_FLOWER_RECOVERY) {
            return;
        }
        // Let the final completed OUT pass use the shorter secondary-shot budget.
        if (state == AutoState.ALT_FLOWER_PICKUP_OUT
                && alternateFlowerCyclesCompleted + 1 >= settings.alternateFlowerPickupCycles
                && remainingMs >= secondaryShotBudgetMs()
                && pathFinished() && alternateFlowerExit.isClear(follower.pose())) {
            return;
        }

        double exitReserve = 0;
        if (activeFlowerExit == primaryFlowerExit) {
            exitReserve = settings.flowerExitTimeoutMs;
        } else if (activeFlowerExit == alternateFlowerExit) {
            exitReserve = settings.alternateFlowerRecoveryTimeoutMs;
        }
        double parkReserve = isSecondaryShotState()
                ? settings.secondaryParkReserveMs : settings.alternateParkReserveMs;
        if (remainingMs <= parkReserve + exitReserve) {
            beginPark();
        }
    }

    private void beginPark() {
        Pose current = follower.pose();
        if (activeFlowerExit != null && !activeFlowerExit.isClear(current)) {
            if (activeFlowerExit == primaryFlowerExit) {
                beginFlowerExit();
            } else {
                beginAlternateFlowerRecovery();
            }
            return;
        }

        activeFlowerExit = null;
        stopFeeding();
        closeShooter();
        intake_servo.setPosition(settings.intakeServoIn);
        flywheel_motor.setPower(0);
        if (current.y() < settings.hiveClearancePose.y()) {
            followLine(settings.hiveClearancePose);
            setState(AutoState.PARK_CLEAR_HIVE);
        } else {
            followLine(parking.targetFrom(current, settings.parkOne, settings.parkTwo));
            setState(AutoState.PARK);
        }
    }

    private boolean pathFinished() {
        return follower.mode() != Follower.Mode.FOLLOW;
    }

    private boolean near(Pose target, double tolerance) {
        return HiveUpRouteChecks.near(follower.pose(), target, tolerance);
    }

    private boolean atShot(Pose target) {
        return HiveUpRouteChecks.atShot(follower.pose(), target,
                settings.shotPositionTolerance, settings.shotHeadingTolerance);
    }

    private void stopInsideFlower() {
        follower.stop();
        stopFeeding();
        closeShooter();
        flywheel_motor.setPower(0);
        secondShotStatus = "Flower exit failed - arm held DOWN";
        setState(AutoState.DONE);
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

    // =========================================
    // SHOOTING
    // =========================================

    private void beginVerifiedShot(AutoState shootingState, Pose target) {
        if (atShot(target)) {
            beginShooting(shootingState);
        } else {
            secondShotStatus = "Drifted from shooting pose - parking";
            beginPark();
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

    // =========================================
    // INTAKE / TRANSFER
    // =========================================

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

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop(boolean saveEndPose) {

        if (follower != null) {

            if (saveEndPose) {
                Pose endPose = follower.pose();
                OpModeStorage.saveAutonomousEnd(endPose,
                        activeFlowerExit != null && !activeFlowerExit.isClear(endPose)
                                ? activeFlowerExit : null, settings.intakeServoOut);
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

        if (intake_servo != null && settings != null
                && (activeFlowerExit == null
                    || (follower != null && activeFlowerExit.isClear(follower.pose())))) {
            intake_servo.setPosition(settings.intakeServoIn);
        }

        if (shooter_servo != null
                && shooter_servo2 != null && settings != null) {

            closeShooter();
        }
    }

}
