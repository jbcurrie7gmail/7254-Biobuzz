
package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red - Hive Up")
public class red_hiveup extends OpMode {

    // =========================================
    // FLYWHEEL CONSTANTS
    // =========================================

    private static final double FLYWHEEL_VELOCITY = 1620;

    private static final double FLYWHEEL_P = 270;
    private static final double FLYWHEEL_I = 0;
    private static final double FLYWHEEL_D = 0;
    private static final double FLYWHEEL_F = 12.3;

    private static final double FLYWHEEL_WAIT_MS = 3000;
    private static final double VELOCITY_TOLERANCE = 25;
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
    private static final double TRANSFER_SHOOT_POWER = 0.75;

    // =========================================
    // DRIVING SPEED CONSTANTS
    // =========================================

    private static final double APPROACH_SPEED = 30.0;
    private static final double SLOW_PICKUP_SPEED = 15.0;

    private static final double PICKUP_DWELL_MS = 500;

    // =========================================
    // AUTONOMOUS TIMEOUT CONSTANTS
    // =========================================

    // Maximum time for robot rotation
    private static final double ROTATE_TIMEOUT_MS = 3000;

    // Travel toward the ball pickup area
    private static final double APPROACH_TIMEOUT_MS = 4000;

    // Slow final corner pickup movement
    private static final double PICKUP_TIMEOUT_MS = 3000;

    // Return from corner to shooting position
    private static final double RETURN_TIMEOUT_MS = 4000;

    // Move 24 inches in negative X
    private static final double STRAFE_TIMEOUT_MS = 3000;

    // Longer travel to parking location
    private static final double PARK_TIMEOUT_MS = 8000;

    // =========================================
    // SERVO POSITIONS
    // =========================================

    // Intake servo remains IN during entire auto
    private static final double INTAKE_SERVO_IN = 0.2;

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
            new Pose(26, 6, Math.toRadians(180));

    private final Pose pickupPose =
            new Pose(7.75, 5, Math.toRadians(180));

    private final Pose strafePose =
            new Pose(32, 7.75, Math.toRadians(270));

    private final Pose parkPose =
            new Pose(12, 93, Math.toRadians(270));

    // =========================================
    // HARDWARE
    // =========================================

    private Follower follower;

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

    // Timeout tracking for telemetry
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
        FINISH_PICKUP,

        RETURN_TO_SHOOT,
        WAIT_SECOND_FLYWHEEL,
        SHOOT_SECOND,

        STRAFE_LEFT,
        PARK,
        DONE
    }

    private AutoState state = AutoState.WAIT_FIRST_FLYWHEEL;

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

        // Motor modes
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

        // Intake servo stays IN
        intake_servo.setPosition(INTAKE_SERVO_IN);

        closeShooter();

        // Pedro Pathing
        follower = Constants.create(hardwareMap);
        follower.setPose(shootPose);

        telemetry.addData("Status", "Initialized");
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

        // Clear previously stored autonomous pose
        OpModeStorage.autonomousEndPose = null;

        timeoutCount = 0;
        lastTimeout = "None";

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

        // Maintain flywheel velocity until finished
        if (state != AutoState.DONE) {
            flywheel_motor.setVelocity(FLYWHEEL_VELOCITY);
        }

        switch (state) {

            // =================================
            // FIRST FLYWHEEL SPINUP
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

            // =================================
            // FIRST SHOOTING SEQUENCE
            // =================================

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

                    setState(AutoState.FINISH_PICKUP);
                }

                break;

            // =================================
            // FINISH PICKUP
            // =================================

            case FINISH_PICKUP:

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

                    setState(
                            AutoState.WAIT_SECOND_FLYWHEEL
                    );
                }

                break;

            // =================================
            // SECOND FLYWHEEL READINESS
            // =================================

            case WAIT_SECOND_FLYWHEEL:

                stopFeeding();
                closeShooter();

                if (flywheelReady()) {

                    beginShooting(AutoState.SHOOT_SECOND);

                } else if (stateTimer.milliseconds()
                        >= FLYWHEEL_WAIT_MS) {

                    recordTimeout();
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

                    stopFeeding();
                    closeShooter();

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

                break;

            // =================================
            // STRAFE NEGATIVE X
            // =================================

            case STRAFE_LEFT:

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

        telemetry.addData(
                "Flywheel Target",
                FLYWHEEL_VELOCITY
        );

        telemetry.addData(
                "Flywheel Actual",
                flywheel_motor.getVelocity()
        );

        telemetry.addData(
                "Flywheel Error",
                FLYWHEEL_VELOCITY
                        - flywheel_motor.getVelocity()
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
                "Follower Mode",
                follower.mode()
        );

        telemetry.addData("Timeout Count", timeoutCount);
        telemetry.addData("Last Timeout", lastTimeout);

        telemetry.update();
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

        // Normal path completion
        if (!follower.isBusy()) {
            return true;
        }

        // Maximum time exceeded
        if (stateTimer.milliseconds() >= timeoutMs) {

            recordTimeout();

            // Cancel unfinished route
            follower.stop();

            // Allow next state to begin
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

        // Intake servo remains IN at 0.2

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

        // Save the robot's ACTUAL ending position
        // before stopping the follower
        OpModeStorage.autonomousEndPose = follower.pose();

        // Stop all motors and path following
        follower.stop();

        flywheel_motor.setPower(0);

        stopFeeding();

        closeShooter();
    }
}
