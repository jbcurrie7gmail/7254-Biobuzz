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
    private final double intake_servo_in = 0.2;
    private final double intake_servo_out = 0.76;

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

    Path toTarget;

    Timer pathTimer = new Timer();
    Timer shooterTimer = new Timer();

    boolean isAutoDrive = false;
    boolean driverOverride = false;


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

        intake_servo.setPosition(intake_servo_in);
        is_intake_servo_in = true;

        // Pedro Pathing initialization
        isAutoDrive = false;
        driverOverride = false;

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
        // Start automatic driving
        if (gamepad1.dpadDownWasPressed() && !isAutoDrive) {

            Pose startPose = follower.pose();

            toTarget = line(startPose, flowerA)
                    .linear(startPose, flowerA);

            follower.follow(toTarget);

            isAutoDrive = true;
            pathTimer.reset();
            driverOverride = false;
        }

        // Check for driver override
        driverOverride =
                Math.abs(gamepad1.left_stick_x) > 0.2 ||
                        Math.abs(gamepad1.left_stick_y) > 0.2 ||
                        Math.abs(gamepad1.right_stick_x) > 0.2;

        // End automatic driving after completion, timeout, or override
        if (isAutoDrive &&
                (!follower.isBusy() ||
                        pathTimer.seconds() > 4 ||
                        driverOverride)) {

            follower.stop();
            isAutoDrive = false;
        }

        // Manual field-centric driving
        if (!isAutoDrive) {

            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            );

            follower.manual(powers);
        }

        // Intake motor control
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
            intake_motor.setPower(-.5);
            transfer_motor.setPower(-.45);
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

        // Update Pedro Pathing
        follower.update();

        // Telemetry
        Pose robotPose = follower.pose();

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading",
                Math.toDegrees(robotPose.heading()));

        telemetry.addData("Auto Drive", isAutoDrive);
        telemetry.addData("Driver Override", driverOverride);
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Intake Servo In", is_intake_servo_in);
        telemetry.update();
    }
}