package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Runs the Two Tips alternate-side sequence from its first shooting pose. */
@Autonomous(name = "Hive Down", preselectTeleOp = "TeleOp_fieldCentric")
public class HiveDown extends OpMode {
    private static final double AUTONOMOUS_SECONDS = 30;

    private static final class Settings {
        Pose start;
        Pose firstShot;
        double firstHeadingTolerance;
        Pose flowerClearance;
        Pose flowerOut;
        Pose flowerIn;
        Pose secondShot;
        Vector2D parkOne;
        Vector2D parkTwo;

        double firstVelocity;
        double secondVelocity;
        double velocityTolerance;
        double velocityStableMs;
        double firstSettleMs;
        double firstSpinupMs;
        double secondSpinupMs;
        double firstShotMs;
        double secondShotMs;
        double gateDelayMs;
        double transferShootPower;

        int flowerCycles;
        double flowerSpeed;
        double flowerTolerance;
        double flowerExitMargin;
        double clearanceTimeoutMs;
        double approachTimeoutMs;
        double pickupInTimeoutMs;
        double pickupOutTimeoutMs;
        double recoveryTimeoutMs;
        double secondTravelTimeoutMs;
        double parkTimeoutMs;
        double startParkAtSeconds;

        double intakeUp;
        double intakeDown;

        double flywheelP;
        double flywheelI;
        double flywheelD;
        double flywheelF;
        double intakePickupPower;
        double transferPickupPower;
        double shooterOpen;
        double shooterClosed;
        double shooter2Open;
        double shooter2Closed;
        int limelightPollHz;
    }

    private enum State {
        FIRST_SPINUP,
        SHOOT_FIRST,
        CLEAR_HIVE,
        APPROACH_FLOWER,
        PICKUP_IN,
        PICKUP_OUT,
        RECOVER_FLOWER_EXIT,
        DRIVE_TO_SECOND_SHOT,
        SECOND_SPINUP,
        SHOOT_SECOND,
        PARKING,
        DONE
    }

    private final Settings blueSettings = createBlueSettings();
    private final Settings redSettings = createRedSettings();
    private Settings settings;
    private OpModeStorage.Alliance alliance = OpModeStorage.Alliance.BLUE;
    private Follower follower;
    private AutonInitMenu menu;
    private AutonParking parking;
    private Limelight3A limelight;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx flywheelMotor;
    private Servo intakeServo;
    private Servo shooterServo;
    private Servo shooterServo2;
    private final Timer stateTimer = new Timer();
    private final Timer velocityTimer = new Timer();
    private State state = State.FIRST_SPINUP;
    private State lastTimeout;
    private boolean speedStable;
    private int completedCycles;
    private HiveUpRouteChecks.FlowerExit flowerExit;
    private boolean flowerArmDown;

    private static Settings createBlueSettings() {
        Settings settings = new Settings();
        settings.start = new Pose(59, 132, Math.toRadians(90));
        settings.firstShot = new Pose(59, 132, Math.toRadians(88));
        settings.firstHeadingTolerance = Math.toRadians(1);
        settings.flowerClearance = new Pose(59, 117, Math.toRadians(88));
        settings.flowerOut = new Pose(48, 122, Math.toRadians(90));
        settings.flowerIn = new Pose(48, 126.5, Math.toRadians(90));
        settings.secondShot = new Pose(33, 122, Math.toRadians(120));
        settings.parkOne = Vector2D.cartesian(12, 112);
        settings.parkTwo = Vector2D.cartesian(12, 93);

        settings.firstVelocity = 1620;
        settings.secondVelocity = 1600;
        settings.velocityTolerance = 20;
        settings.velocityStableMs = 200;
        settings.firstSettleMs = 1000;
        settings.firstSpinupMs = 3000;
        settings.secondSpinupMs = 750;
        settings.firstShotMs = 1500;
        settings.secondShotMs = 1000;
        settings.gateDelayMs = 150;
        settings.transferShootPower = 0.60;

        settings.flowerCycles = 3;
        settings.flowerSpeed = 20;
        settings.flowerTolerance = 2;
        settings.flowerExitMargin = 1;
        settings.clearanceTimeoutMs = 2000;
        settings.approachTimeoutMs = 2500;
        settings.pickupInTimeoutMs = 1500;
        settings.pickupOutTimeoutMs = 2000;
        settings.recoveryTimeoutMs = 1800;
        settings.secondTravelTimeoutMs = 3000;
        settings.parkTimeoutMs = 8000;
        settings.startParkAtSeconds = 27;

        settings.intakeUp = 0.18;
        settings.intakeDown = 0.69;

        settings.flywheelP = 270;
        settings.flywheelI = 0;
        settings.flywheelD = 0;
        settings.flywheelF = 12.3;
        settings.intakePickupPower = 1.0;
        settings.transferPickupPower = 0.5;
        settings.shooterOpen = 0.5;
        settings.shooterClosed = 0.88;
        settings.shooter2Open = 0.88;
        settings.shooter2Closed = 0.5;
        settings.limelightPollHz = 100;
        return settings;
    }

    private static Settings createRedSettings() {
        Settings settings = new Settings();
        settings.start = new Pose(59, 132, Math.toRadians(90));
        settings.firstShot = new Pose(59, 132, Math.toRadians(88));
        settings.firstHeadingTolerance = Math.toRadians(1);
        settings.flowerClearance = new Pose(59, 117, Math.toRadians(88));
        settings.flowerOut = new Pose(48, 122, Math.toRadians(90));
        settings.flowerIn = new Pose(48, 126.5, Math.toRadians(90));
        settings.secondShot = new Pose(33, 122, Math.toRadians(120));
        settings.parkOne = Vector2D.cartesian(12, 112);
        settings.parkTwo = Vector2D.cartesian(12, 93);

        settings.firstVelocity = 1620;
        settings.secondVelocity = 1600;
        settings.velocityTolerance = 20;
        settings.velocityStableMs = 200;
        settings.firstSettleMs = 1000;
        settings.firstSpinupMs = 3000;
        settings.secondSpinupMs = 750;
        settings.firstShotMs = 1500;
        settings.secondShotMs = 1000;
        settings.gateDelayMs = 150;
        settings.transferShootPower = 0.60;

        settings.flowerCycles = 3;
        settings.flowerSpeed = 20;
        settings.flowerTolerance = 2;
        settings.flowerExitMargin = 1;
        settings.clearanceTimeoutMs = 2000;
        settings.approachTimeoutMs = 2500;
        settings.pickupInTimeoutMs = 1500;
        settings.pickupOutTimeoutMs = 2000;
        settings.recoveryTimeoutMs = 1800;
        settings.secondTravelTimeoutMs = 3000;
        settings.parkTimeoutMs = 8000;
        settings.startParkAtSeconds = 27;

        settings.intakeUp = 0.18;
        settings.intakeDown = 0.69;

        settings.flywheelP = 270;
        settings.flywheelI = 0;
        settings.flywheelD = 0;
        settings.flywheelF = 12.3;
        settings.intakePickupPower = 1.0;
        settings.transferPickupPower = 0.5;
        settings.shooterOpen = 0.5;
        settings.shooterClosed = 0.88;
        settings.shooter2Open = 0.88;
        settings.shooter2Closed = 0.5;
        settings.limelightPollHz = 100;
        return settings;
    }

    @Override
    public void init() {
        menu = new AutonInitMenu(false);
        alliance = menu.alliance();
        parking = new AutonParking();
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel_motor");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        shooterServo = hardwareMap.get(Servo.class, "shooter_servo");
        shooterServo2 = hardwareMap.get(Servo.class, "shooter_servo2");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setPower(0);

        follower = Constants.create(hardwareMap);
        applySettings(blueSettings);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        showInitSelection();
    }

    @Override
    public void init_loop() {
        menu.update(gamepad2);
        parking.selectParkOne(menu.parkOne());
        alliance = menu.alliance();
        applySettings(alliance == OpModeStorage.Alliance.RED ? redSettings : blueSettings);
        showInitSelection();
    }

    private void applySettings(Settings selected) {
        if (settings == selected) {
            return;
        }
        settings = selected;
        configureFlowerExit();
        flywheelMotor.setVelocityPIDFCoefficients(
                settings.flywheelP, settings.flywheelI, settings.flywheelD, settings.flywheelF);
        stopFeeding();
        raiseIntake();
        follower.setPose(settings.start);
    }

    private void showInitSelection() {
        menu.show(telemetry);
        parking.show(telemetry, settings.parkOne, settings.parkTwo);
        telemetry.addData("Start pose", "X %.1f, Y %.1f, heading %.0f",
                settings.start.x(), settings.start.y(), Math.toDegrees(settings.start.heading()));
        telemetry.addData("Limelight pipeline", alliance.aprilTagPipeline);
        telemetry.update();
    }

    @Override
    public void start() {
        OpModeStorage.beginAutonomous(alliance);
        limelight.setPollRateHz(settings.limelightPollHz);
        limelight.pipelineSwitch(alliance.aprilTagPipeline);
        limelight.start();
        completedCycles = 0;
        lastTimeout = null;
        resetRuntime();
        follower.hold(settings.firstShot);
        flywheelMotor.setVelocity(settings.firstVelocity);
        setState(State.FIRST_SPINUP);
    }

    @Override
    public void loop() {
        follower.update();
        double elapsed = getRuntime();
        if (elapsed >= settings.startParkAtSeconds
                || (isPickingUpFlower()
                    && elapsed >= settings.startParkAtSeconds - settings.recoveryTimeoutMs / 1000.0)) {
            prioritizePark();
        }

        switch (state) {
            case FIRST_SPINUP:
                boolean firstReady = flywheelReady(settings.firstVelocity);
                double headingError = settings.firstShot.heading() - follower.pose().heading();
                boolean firstAligned = Math.abs(Math.atan2(Math.sin(headingError),
                        Math.cos(headingError))) <= settings.firstHeadingTolerance;
                if (stateTimer.milliseconds() >= settings.firstSpinupMs
                        || (stateTimer.milliseconds() >= settings.firstSettleMs && firstReady && firstAligned)) {
                    beginShot(State.SHOOT_FIRST);
                }
                break;

            case SHOOT_FIRST:
                feedShot();
                if (stateTimer.milliseconds() >= settings.firstShotMs) {
                    stopFeeding();
                    flywheelMotor.setPower(0);
                    moveTo(settings.flowerClearance, State.CLEAR_HIVE);
                }
                break;

            case CLEAR_HIVE:
                if (pathFinished() && near(settings.flowerClearance)) {
                    intakeServo.setPosition(settings.intakeDown);
                    flowerArmDown = true;
                    moveTo(settings.flowerOut, State.APPROACH_FLOWER);
                } else if (timedOut(settings.clearanceTimeoutMs)) {
                    beginPark();
                }
                break;

            case APPROACH_FLOWER:
                if (pathFinished() && near(settings.flowerOut)) {
                    startPickup();
                    moveTo(settings.flowerIn, settings.flowerSpeed, State.PICKUP_IN);
                } else if (timedOut(settings.approachTimeoutMs)) {
                    beginRecovery();
                }
                break;

            case PICKUP_IN:
                if (pathFinished() && near(settings.flowerIn)) {
                    moveTo(flowerExit.target, settings.flowerSpeed, State.PICKUP_OUT);
                } else if (timedOut(settings.pickupInTimeoutMs) || pathFinished()) {
                    beginRecovery();
                }
                break;

            case PICKUP_OUT:
                if (pathFinished() && flowerIsClear()) {
                    completedCycles++;
                    if (completedCycles < settings.flowerCycles) {
                        moveTo(settings.flowerIn, settings.flowerSpeed, State.PICKUP_IN);
                    } else {
                        stopFeeding();
                        raiseIntake();
                        flywheelMotor.setVelocity(settings.secondVelocity);
                        moveTo(settings.secondShot, State.DRIVE_TO_SECOND_SHOT);
                    }
                } else if (timedOut(settings.pickupOutTimeoutMs)) {
                    beginRecovery();
                }
                break;

            case RECOVER_FLOWER_EXIT:
                if (flowerIsClear()) {
                    beginPark();
                } else if (elapsed >= AUTONOMOUS_SECONDS) {
                    follower.stop();
                    stopFeeding();
                    setState(State.DONE);
                } else if (timedOut(settings.recoveryTimeoutMs)) {
                    beginRecovery();
                }
                break;

            case DRIVE_TO_SECOND_SHOT:
                if (pathFinished() && near(settings.secondShot)) {
                    setState(State.SECOND_SPINUP);
                } else if (timedOut(settings.secondTravelTimeoutMs)) {
                    beginPark();
                }
                break;

            case SECOND_SPINUP:
                if (flywheelReady(settings.secondVelocity) || stateTimer.milliseconds() >= settings.secondSpinupMs) {
                    beginShot(State.SHOOT_SECOND);
                }
                break;

            case SHOOT_SECOND:
                feedShot();
                if (stateTimer.milliseconds() >= settings.secondShotMs) {
                    beginPark();
                }
                break;

            case PARKING:
                if (pathFinished() || timedOut(settings.parkTimeoutMs)) {
                    follower.stop();
                    setState(State.DONE);
                }
                break;

            case DONE:
                break;
        }

        Pose pose = follower.pose();
        telemetry.addData("Hive Down", state);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose", "X %.1f, Y %.1f, heading %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Flower cycles", "%d / %d", completedCycles, settings.flowerCycles);
        telemetry.addData("Flywheel velocity", flywheelMotor.getVelocity());
        telemetry.addData("Last timeout", lastTimeout);
        telemetry.update();
    }

    private boolean isPickingUpFlower() {
        return state == State.APPROACH_FLOWER || state == State.PICKUP_IN
                || state == State.PICKUP_OUT;
    }

    private void prioritizePark() {
        switch (state) {
            case APPROACH_FLOWER:
            case PICKUP_IN:
            case PICKUP_OUT:
                beginRecovery();
                break;
            case RECOVER_FLOWER_EXIT:
            case PARKING:
            case DONE:
                break;
            default:
                beginPark();
                break;
        }
    }

    private void configureFlowerExit() {
        flowerExit = new HiveUpRouteChecks.FlowerExit(settings.flowerIn, settings.flowerOut,
                settings.flowerExitMargin, settings.flowerTolerance);
    }

    private boolean flowerIsClear() {
        return flowerExit.isClear(follower.pose());
    }

    private void raiseIntake() {
        intakeServo.setPosition(settings.intakeUp);
        flowerArmDown = false;
    }

    private void beginRecovery() {
        startPickup();
        moveTo(flowerExit.target, settings.flowerSpeed, State.RECOVER_FLOWER_EXIT);
    }

    private void beginPark() {
        stopFeeding();
        flywheelMotor.setPower(0);
        raiseIntake();
        Pose target = parking.targetFrom(follower.pose(), settings.parkOne, settings.parkTwo);
        moveTo(target, State.PARKING);
    }

    private void moveTo(Pose target, State next) {
        Pose current = follower.pose();
        follower.follow(line(current, target).linear(current, target));
        setState(next);
    }

    private void moveTo(Pose target, double speed, State next) {
        Pose current = follower.pose();
        follower.follow(line(current, target).linear(current, target)
                .with(Constants.foresightConfig.maxVelocityConstraint.at(speed)));
        setState(next);
    }

    private boolean pathFinished() {
        return follower.mode() != Follower.Mode.FOLLOW;
    }

    private boolean near(Pose target) {
        Pose current = follower.pose();
        return Math.hypot(current.x() - target.x(), current.y() - target.y()) <= settings.flowerTolerance;
    }

    private void setState(State next) {
        state = next;
        stateTimer.reset();
        speedStable = false;
    }

    private boolean timedOut(double timeoutMs) {
        if (stateTimer.milliseconds() < timeoutMs) {
            return false;
        }
        lastTimeout = state;
        return true;
    }

    private boolean flywheelReady(double velocity) {
        if (Math.abs(flywheelMotor.getVelocity() - velocity) > settings.velocityTolerance) {
            speedStable = false;
            return false;
        }
        if (!speedStable) {
            velocityTimer.reset();
            speedStable = true;
        }
        return velocityTimer.milliseconds() >= settings.velocityStableMs;
    }

    private void beginShot(State shootingState) {
        shooterServo.setPosition(settings.shooterOpen);
        shooterServo2.setPosition(settings.shooter2Open);
        setState(shootingState);
    }

    private void feedShot() {
        if (stateTimer.milliseconds() >= settings.gateDelayMs) {
            transferMotor.setPower(settings.transferShootPower);
        }
    }

    private void startPickup() {
        intakeMotor.setPower(settings.intakePickupPower);
        transferMotor.setPower(settings.transferPickupPower);
    }

    private void stopFeeding() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        shooterServo.setPosition(settings.shooterClosed);
        shooterServo2.setPosition(settings.shooter2Closed);
    }

    @Override
    public void stop() {
        if (follower != null) {
            Pose endPose = follower.pose();
            OpModeStorage.saveAutonomousEnd(endPose,
                    flowerArmDown && !flowerExit.isClear(endPose) ? flowerExit : null, settings.intakeDown);
            follower.stop();
        }
        if (limelight != null) {
            limelight.stop();
        }
        if (intakeMotor != null && transferMotor != null
                && shooterServo != null && shooterServo2 != null) {
            stopFeeding();
        }
        if (flywheelMotor != null) {
            flywheelMotor.setPower(0);
        }
    }
}
