package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TeleOp_fieldCentric")
public class teleop_fieldCentric extends OpMode {

    // Intake servo positions
    private final double intake_servo_in = 0.18;
    private final double intake_servo_out = 0.69;

    private double shooter_servo_closed = 0.88;
    private double shooter_servo_open = 0.5;

    private double shooter_servo2_closed = 0.5;
    private double shooter_servo2_open = 0.88;
    private int flywheel_velocity = 1620;

    private boolean is_intake_servo_in = true;
    private Servo intake_servo, shooter_servo, shooter_servo2;
    private DcMotorEx intake_motor, flywheel_motor, transfer_motor;

    // Pedro Pathing
    private Follower follower;

    Pose startPose = new Pose(12, 93, Math.toRadians(270));
    Pose flowerA = new Pose(21.0, 47.0, Math.toRadians(180));
    Pose flowerB = new Pose(49.0, 120.0, Math.toRadians(90));
    Pose flowerC = new Pose(123.0, 92.0, Math.toRadians(0));
    Pose flowerD = new Pose(94.0, 18.0, Math.toRadians(270));

    private static final int FLOWER_PICKUP_CYCLES = 3;
    private static final double FLOWER_EXIT_TOLERANCE = 2.0;
    private static final double FLOWER_APPROACH_SPEED = 30.0;
    private static final double FLOWER_SWEEP_SPEED = 15.0;

    private enum PickupPhase {
        IDLE, APPROACH, IN, OUT
    }

    private static final double PRIMARY_RANGE = 56.43;
    private static final double ALTERNATE_RANGE = 55.87;
    private static final double SHOT_ARC_HALF_ANGLE = Math.toRadians(15);
    private static final double SHOT_HEADING_TOLERANCE = Math.toRadians(2);
    private static final double AIM_TURN_GAIN = 1.8;
    private static final double MIN_AIM_TURN_POWER = 0.12;
    private static final double MAX_AIM_TURN_POWER = 0.55;
    private static final double SHOT_DEADLINE_MS = 1000;
    private static final double SHOT_SETTLE_MS = 300;
    private static final double SHOOTER_SERVO_DELAY_MS = 150;
    private static final double TRIGGER_THRESHOLD = 0.1;

    // Both alliances use the same coordinates from their own hive-up corner.
    private static final ShootingArc PRIMARY_ARC =
            new ShootingArc("Primary", new Pose(56, 7.75, Math.toRadians(270)), PRIMARY_RANGE);
    private static final ShootingArc ALTERNATE_ARC =
            new ShootingArc("Alternate",
                    new Pose(59.1205, 129.5394, Math.toRadians(88.2604)), ALTERNATE_RANGE);

    private static final class ShootingArc {
        final String side;
        final double centerX;
        final double centerY;
        final double referenceHeading;
        final double radius;

        ShootingArc(String side, Pose reference, double calibratedRange) {
            this.side = side;
            referenceHeading = reference.heading();
            centerX = reference.x() + calibratedRange * Math.cos(referenceHeading);
            centerY = reference.y() + calibratedRange * Math.sin(referenceHeading);
            radius = calibratedRange;
        }

        double headingToCenter(double x, double y) {
            return Math.atan2(centerY - y, centerX - x);
        }

        Pose closestPose(Pose robot) {
            double heading = headingToCenter(robot.x(), robot.y());
            double offset = wrapAngle(heading - referenceHeading);
            double closestHeading = referenceHeading
                    + Math.max(-SHOT_ARC_HALF_ANGLE, Math.min(SHOT_ARC_HALF_ANGLE, offset));
            double x = centerX - radius * Math.cos(closestHeading);
            double y = centerY - radius * Math.sin(closestHeading);
            return new Pose(x, y, headingToCenter(x, y));
        }
    }

    private enum ShotPhase {
        IDLE, ALIGNING, SETTLING, SHOOTING, SHOOTING_AT_TIMEOUT
    }

    private ShootingArc activeArc;
    private ShotPhase shotPhase = ShotPhase.IDLE;
    private final Timer aimTimer = new Timer();
    private final Timer shooterTimer = new Timer();
    private final Timer pathTimer = new Timer();
    private PickupPhase pickupPhase = PickupPhase.IDLE;
    private Pose flowerApproach;
    private Pose flowerInside;
    private int flowerCyclesCompleted;

    @Override
    public void init() {

        // Hardware initialization
        intake_motor = hardwareMap.get(
                DcMotorEx.class, "intake_motor"
        );

        transfer_motor = hardwareMap.get(
                DcMotorEx.class, "transfer_motor"
        );

        flywheel_motor = hardwareMap.get(
                DcMotorEx.class, "flywheel_motor"
        );

        intake_servo = hardwareMap.get(
                Servo.class, "intake_servo"
        );

        shooter_servo = hardwareMap.get(
                Servo.class, "shooter_servo"
        );

        shooter_servo2 = hardwareMap.get(
                Servo.class, "shooter_servo2"
        );

        // Pedro Pathing initialization

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel_motor.setVelocityPIDFCoefficients(
                270,  // P
                0,   // I
                0,   // D
                12.3   // F
        );
        transfer_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter_servo2.setPosition(shooter_servo2_closed);
        shooter_servo.setPosition(shooter_servo_closed);
    }
    @Override
    public void start() {
        intake_servo.setPosition(intake_servo_in);
        is_intake_servo_in = true;

        if (OpModeStorage.autonomousEndPose != null) {

            // Use the actual ending pose from autonomous
            follower.setPose(OpModeStorage.autonomousEndPose);

        } else {

            // Fallback if autonomous has not run
            follower.setPose(startPose);
        }

        follower.update();
    }

    @Override
    public void loop() {
        flywheel_motor.setVelocity(flywheel_velocity);
        boolean driverOverride =
                Math.abs(gamepad1.left_stick_x) > 0.2 ||
                Math.abs(gamepad1.left_stick_y) > 0.2 ||
                Math.abs(gamepad1.right_stick_x) > 0.2;

        boolean leftHeld = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        // Both triggers can start reverse intake after the operator releases the shot.
        boolean shotRequested = leftHeld
                && (shotPhase != ShotPhase.IDLE || gamepad2.right_trigger <= TRIGGER_THRESHOLD);
        if (!shotRequested && shotPhase != ShotPhase.IDLE) {
            endShot();
        }
        follower.update();

        if (shotRequested) {
            if (pickupPhase != PickupPhase.IDLE) {
                finishFlowerPickup(false);
            }
            updateShot();
        }

        if (pickupPhase != PickupPhase.IDLE) {
            if (driverOverride) {
                finishFlowerPickup(false);
            } else {
                updateFlowerPickup();
            }
        } else if (!driverOverride && !shotRequested) {
            if (gamepad1.dpadDownWasPressed()) {
                beginFlowerPickup(flowerA);
            } else if (gamepad1.dpadLeftWasPressed()) {
                beginFlowerPickup(flowerB);
            } else if (gamepad1.dpadUpWasPressed()) {
                beginFlowerPickup(flowerC);
            } else if (gamepad1.dpadRightWasPressed()) {
                beginFlowerPickup(flowerD);
            }
        }

        boolean isAutoDrive = pickupPhase != PickupPhase.IDLE || shotRequested;

        // Manual field-centric driving
        if (!isAutoDrive) {

            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            );

            follower.manual(powers);
            updateManualIntake();
        }

        Pose robotPose = follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Auto Drive", isAutoDrive);
        telemetry.addData("Driver Override", driverOverride);
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Flower Pickup", pickupPhase);
        telemetry.addData("Flower Cycles", flowerCyclesCompleted);
        telemetry.addData("Intake Servo In", is_intake_servo_in);
        telemetry.addData("Hive alliance", OpModeStorage.alliance);
        telemetry.addData("Hive shot", shotPhase);
        if (activeArc != null) {
            double desiredHeading = activeArc.headingToCenter(robotPose.x(), robotPose.y());
            double error = wrapAngle(desiredHeading - robotPose.heading());
            telemetry.addData("Aim goal", activeArc.side);
            telemetry.addData("Aim heading", "%.1f deg (error %+.1f deg)",
                    (Math.toDegrees(desiredHeading) + 360) % 360, Math.toDegrees(error));
        }
        telemetry.update();
    }

    private void updateShot() {
        Pose robot = follower.pose();
        if (shotPhase == ShotPhase.IDLE) {
            Pose primary = PRIMARY_ARC.closestPose(robot);
            Pose alternate = ALTERNATE_ARC.closestPose(robot);
            activeArc = distanceBetween(robot, primary) <= distanceBetween(robot, alternate)
                    ? PRIMARY_ARC : ALTERNATE_ARC;
            holdFeederClosed();
            aimTimer.reset();
            shotPhase = ShotPhase.ALIGNING;
        }

        double desiredHeading = activeArc.headingToCenter(robot.x(), robot.y());
        double error = wrapAngle(desiredHeading - robot.heading());
        boolean aligned = Math.abs(error) <= SHOT_HEADING_TOLERANCE;
        boolean deadlineReached = aimTimer.milliseconds() >= SHOT_DEADLINE_MS;
        if (deadlineReached && (shotPhase == ShotPhase.ALIGNING || shotPhase == ShotPhase.SETTLING)) {
            openShooter(ShotPhase.SHOOTING_AT_TIMEOUT);
        }

        double turn = 0;
        switch (shotPhase) {
            case ALIGNING:
                if (aligned) {
                    shooterTimer.reset();
                    shotPhase = ShotPhase.SETTLING;
                } else {
                    double power = Math.min(MAX_AIM_TURN_POWER,
                            Math.max(MIN_AIM_TURN_POWER, Math.abs(error) * AIM_TURN_GAIN));
                    turn = Math.copySign(power, error);
                }
                break;

            case SETTLING:
                if (!aligned) {
                    shotPhase = ShotPhase.ALIGNING;
                } else if (shooterTimer.milliseconds() >= SHOT_SETTLE_MS) {
                    openShooter(ShotPhase.SHOOTING);
                }
                break;

            case SHOOTING:
            case SHOOTING_AT_TIMEOUT:
                if (shotPhase == ShotPhase.SHOOTING && !aligned && !deadlineReached) {
                    holdFeederClosed();
                    shotPhase = ShotPhase.ALIGNING;
                } else if (shooterTimer.milliseconds() >= SHOOTER_SERVO_DELAY_MS) {
                    intake_motor.setPower(1);
                    transfer_motor.setPower(0.75);
                }
                break;
        }

        follower.manual(new DrivePowers(0, 0, turn));
    }

    private void openShooter(ShotPhase shootingPhase) {
        shooter_servo.setPosition(shooter_servo_open);
        shooter_servo2.setPosition(shooter_servo2_open);
        shooterTimer.reset();
        shotPhase = shootingPhase;
    }

    private static double wrapAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private void endShot() {
        follower.stop();
        holdFeederClosed();
        shotPhase = ShotPhase.IDLE;
        activeArc = null;
    }

    private void holdFeederClosed() {
        intake_motor.setPower(0);
        transfer_motor.setPower(0);
        shooter_servo.setPosition(shooter_servo_closed);
        shooter_servo2.setPosition(shooter_servo2_closed);
    }

    private void beginFlowerPickup(Pose approach) {
        Pose current = follower.pose();
        flowerApproach = approach;

        // Use the autonomous inside coordinates for A and B. Use the
        // primary flower's 6.25-inch sweep along the heading for C and D.
        double sweepDistance = 6.25;
        if (flowerApproach == flowerA) {
            sweepDistance = flowerA.x() - 12.75;
        } else if (flowerApproach == flowerB) {
            sweepDistance = 126.5 - flowerB.y();
        }
        double heading = flowerApproach.heading();
        flowerInside = new Pose(
                flowerApproach.x() + sweepDistance * Math.cos(heading),
                flowerApproach.y() + sweepDistance * Math.sin(heading),
                heading);
        flowerCyclesCompleted = 0;

        shooter_servo.setPosition(shooter_servo_closed);
        shooter_servo2.setPosition(shooter_servo2_closed);
        intake_servo.setPosition(intake_servo_out);
        is_intake_servo_in = false;
        intake_motor.setPower(1.0);
        transfer_motor.setPower(0.5);

        double headingError = current.heading() - heading;
        double wrappedError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
        if (distanceBetween(current, flowerApproach) <= FLOWER_EXIT_TOLERANCE
                && Math.abs(wrappedError) <= Math.toRadians(10)) {
            followFlowerLeg(flowerInside, PickupPhase.IN, FLOWER_SWEEP_SPEED);
        } else {
            followFlowerLeg(flowerApproach, PickupPhase.APPROACH, FLOWER_APPROACH_SPEED);
        }
    }

    private void updateFlowerPickup() {
        double timeout = pickupPhase == PickupPhase.APPROACH ? 4.0 : 2.0;
        if (follower.isBusy()) {
            if (pathTimer.seconds() > timeout) {
                finishFlowerPickup(false);
            }
            return;
        }

        Pose target = pickupPhase == PickupPhase.IN ? flowerInside : flowerApproach;
        if (distanceBetween(follower.pose(), target) > FLOWER_EXIT_TOLERANCE) {
            finishFlowerPickup(false);
            return;
        }

        switch (pickupPhase) {
            case APPROACH:
                followFlowerLeg(flowerInside, PickupPhase.IN, FLOWER_SWEEP_SPEED);
                break;
            case IN:
                followFlowerLeg(flowerApproach, PickupPhase.OUT, FLOWER_SWEEP_SPEED);
                break;
            case OUT:
                flowerCyclesCompleted++;
                if (flowerCyclesCompleted == FLOWER_PICKUP_CYCLES) {
                    finishFlowerPickup(true);
                } else {
                    followFlowerLeg(flowerInside, PickupPhase.IN, FLOWER_SWEEP_SPEED);
                }
                break;
        }
    }

    private void followFlowerLeg(Pose target, PickupPhase phase, double speed) {
        Pose current = follower.pose();
        Path path = line(current, target)
                .linear(current, target)
                .with(Constants.foresightConfig.maxVelocityConstraint.at(speed));
        follower.follow(path);
        pickupPhase = phase;
        pathTimer.reset();
    }

    private static double distanceBetween(Pose first, Pose second) {
        return Math.hypot(first.x() - second.x(), first.y() - second.y());
    }

    private void finishFlowerPickup(boolean completed) {
        follower.stop();
        pickupPhase = PickupPhase.IDLE;
        intake_motor.setPower(0);
        transfer_motor.setPower(0);
        shooterTimer.reset();
        if (completed) {
            intake_servo.setPosition(intake_servo_in);
            is_intake_servo_in = true;
        }
    }

    @Override
    public void stop() {
        if (pickupPhase != PickupPhase.IDLE) {
            finishFlowerPickup(false);
        }
        if (shotPhase != ShotPhase.IDLE) {
            endShot();
        }
        holdFeederClosed();
        flywheel_motor.setPower(0);
    }

    private void updateManualIntake() {
        shooter_servo.setPosition(shooter_servo_closed);
        shooter_servo2.setPosition(shooter_servo2_closed);

        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
                intake_motor.setPower(gamepad2.a ? -1 : -0.5);
                transfer_motor.setPower(gamepad2.a ? -1 : -0.45);
            } else {
                intake_motor.setPower(1);
                transfer_motor.setPower(0.5);
            }
        } else {
            intake_motor.setPower(0);
            transfer_motor.setPower(0);
        }

        if (gamepad2.yWasPressed()) {
            is_intake_servo_in = !is_intake_servo_in;
            intake_servo.setPosition(is_intake_servo_in ? intake_servo_in : intake_servo_out);
        }
    }
}
