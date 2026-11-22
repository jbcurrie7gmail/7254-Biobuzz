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

    private final Timer pathTimer = new Timer();
    private final Timer shooterTimer = new Timer();
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

        follower.update();
        if (pickupPhase != PickupPhase.IDLE) {
            if (driverOverride) {
                finishFlowerPickup(false);
            } else {
                updateFlowerPickup();
            }
        } else if (!driverOverride) {
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

        boolean isAutoDrive = pickupPhase != PickupPhase.IDLE;

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
        telemetry.update();
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
    }

    private void updateManualIntake() {
        if (gamepad2.right_trigger > 0.1 && gamepad2.left_trigger < 0.1) {
            shooter_servo.setPosition(shooter_servo_closed);
            shooter_servo2.setPosition(shooter_servo2_closed);
            intake_motor.setPower(1);
            transfer_motor.setPower(.5);
            shooterTimer.reset();
        }


        if(gamepad2.left_trigger > 0.1 && gamepad2.right_trigger < 0.1)
        {
           shooter_servo.setPosition(shooter_servo_open);
           shooter_servo2.setPosition(shooter_servo2_open);
           if(shooterTimer.milliseconds()>150) {
               transfer_motor.setPower(.75);
               intake_motor.setPower(1);
           }
        }

        if(gamepad2.left_trigger < 0.1 && gamepad2.right_trigger < 0.1)
        {
            intake_motor.setPower(0);
            transfer_motor.setPower(0);
            shooter_servo.setPosition(shooter_servo_closed);
            shooter_servo2.setPosition(shooter_servo2_closed);
            shooterTimer.reset();
        }

        if(gamepad2.left_trigger > 0.1 && gamepad2.right_trigger > 0.1)
        {
            if(gamepad2.a)
            {
                intake_motor.setPower(-1);
                transfer_motor.setPower(-1);
            }
            else {
                intake_motor.setPower(-.5);
                transfer_motor.setPower(-.45);
            }
            shooter_servo.setPosition(shooter_servo_closed);
            shooter_servo2.setPosition(shooter_servo2_closed);
            shooterTimer.reset();
        }



        // Intake servo toggle (no oscillation)
        if (gamepad2.yWasPressed()) {

            is_intake_servo_in = !is_intake_servo_in;

            if (is_intake_servo_in) {
                intake_servo.setPosition(intake_servo_in);
            } else {
                intake_servo.setPosition(intake_servo_out);
            }
        }


    }
}