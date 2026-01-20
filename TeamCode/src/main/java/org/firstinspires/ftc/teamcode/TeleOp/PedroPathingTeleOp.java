package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.mechanisms.DigitalSensor;
import org.firstinspires.ftc.teamcode.mechanisms.FollowerPose;
import org.firstinspires.ftc.teamcode.mechanisms.IndicatorLight;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

/**
 * TeleOp program for an FTC robot with:
 *  - Four-motor Mecanum wheel drivetrain
 *  - One launcher motor using velocity control
 *  - Two feeder CRServos
 *  - Intake motor
 * CONTROLS:
 *  Left stick Y = forward/backward (up = forward, down = backward)
 *  Left stick X = strafe (left = left, right = right)
 *  Right stick X = rotate (left = CCW, right = CW)
 *  Right bumper = launcher to fire one close shot
 *  Right trigger = launcher to fire one far shot
 *  Left bumper  = intake forward
 *  Left trigger = intake stop
 *  D-pad Left   = intake reverse
 */

public class PedroPathingTeleOp extends OpMode {
    Follower follower;
    FollowerPose followerPose;
    Pose currentPose;

    // Use Pedro pathing Mecanum drivetrain
    Mecanum mecanum;
    List<DcMotorEx> motors;
    double[] motorPower;

    // Intake motor to grab artifacts in the field
    IntakeMotor intakeMotor = new IntakeMotor()
            .motorName("intake")
            .motorUseBrakeMode(true)
            .motorDirection(DcMotorSimple.Direction.REVERSE);

    // Launcher and feeders to shoot artifacts
    Launcher launcher = new Launcher()
            .launcherName("launcher")
            .launcherUseBrakeMode(true)
            .launcherDirection(DcMotorSimple.Direction.REVERSE)
            .leftFeederName("left_feeder")
            .rightFeederName("right_feeder")
            .leftFeederDirection(DcMotorSimple.Direction.FORWARD)
            .rightFeederDirection(DcMotorSimple.Direction.REVERSE);

    // Indicator light
    IndicatorLight indicatorLight = new IndicatorLight()
            .indicatorName("indicator_light");
    boolean indicateDetected = false;

    // Sensor to detect whether the artifact is present for launching
    DigitalSensor leftBallSensor = new DigitalSensor()
            .sensorName("left_ball_sensor")
            .sensorMode(DigitalChannel.Mode.INPUT);
    DigitalSensor rightBallSensor = new DigitalSensor()
            .sensorName("right_ball_sensor")
            .sensorMode(DigitalChannel.Mode.INPUT);
    Timer sensorTime = new Timer();

    // Drivetrain constants
    double DRIVE_MAX_POWER = 1.0;
    double DRIVE_MAX_FORWARD_SPEED = 1.0;
    double DRIVE_MAX_ANGULAR_SPEED = 0.7;

    // Intake motor constants
    double INTAKE_POWER = 0.75;
    double INTAKE_PANIC_TIME = 0.05;

    // Launcher constants
    double ClOSE_LAUNCH_TARGET_VELOCITY = 1300;
    double CLOSE_LAUNCH_MIN_VELOCITY    = 1280;
    double FAR_LAUNCH_TARGET_VELOCITY   = 1600;
    double FAR_LAUNCH_MIN_VELOCITY      = 1580;

    double FEEDER_RUN_SECONDS = 0.10;
    double FEEDER_PANIC_INTERVAL = 0.10 + FEEDER_RUN_SECONDS;
    double LAUNCH_COOL_OFF_SECONDS = 0.20;

    // OpMode timer
    Timer runTime = new Timer();
    boolean isAlmostEndGame = false;

    // Alliance enum and misc.
    enum Alliance { BLUE, RED } Alliance alliance;
    boolean isRobotCentric, isIMUResetRequested;
    double driveHeadingOffset;

    @Override
    public void init() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);

        // Initialize Mecanum drivetrain
        mecanum = (Mecanum) follower.getDrivetrain();
        mecanum.setMaxPowerScaling(DRIVE_MAX_POWER);

        motors = mecanum.getMotors();
        motorPower = new double[motors.size()];

        // Initialize intake motor
        intakeMotor.build(hardwareMap);
        intakeMotor.setPower(INTAKE_POWER);
        intakeMotor.setPanicTime(INTAKE_PANIC_TIME);

        // Initialize launcher and feeders
        launcher.build(hardwareMap);
        launcher.launcherOffAtIdle();
        launcher.setLauncherCloseVelocity(ClOSE_LAUNCH_TARGET_VELOCITY, CLOSE_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherFarVelocity(FAR_LAUNCH_TARGET_VELOCITY, FAR_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherCoolOffSec(LAUNCH_COOL_OFF_SECONDS);
        launcher.setFeederRunSec(FEEDER_RUN_SECONDS);

        // Initialize indicator light
        indicatorLight.build(hardwareMap);

        // Initialize Digital sensor
        leftBallSensor.build(hardwareMap);
        rightBallSensor.build(hardwareMap);

        if (isIMUResetRequested) {
            if (alliance == Alliance.BLUE)
                followerPose = new FollowerPose("blue");
            else if (alliance == Alliance.RED)
                followerPose = new FollowerPose("red");
            else throw new IllegalArgumentException("Illegal alliance assignment");
            followerPose.useFarStartPose();
            follower.setStartingPose(followerPose.startPose);
        }
        follower.update();
        currentPose = follower.getPose();

        if (isRobotCentric) {
            driveHeadingOffset = 0.0;
        } else {
            if (alliance == Alliance.BLUE) {
                driveHeadingOffset = Math.PI - (isIMUResetRequested ? Math.PI/2 : currentPose.getHeading());
            } else if (alliance == Alliance.RED) {
                driveHeadingOffset = 2*Math.PI - (isIMUResetRequested ? Math.PI/2 : currentPose.getHeading());
            } else throw new IllegalArgumentException("Illegal alliance assignment");
        }

        telemetry.addData("Alliance Team", alliance.toString());
        telemetry.addData("Use Orientation", isRobotCentric ? "Robot centric" : "Field centric");
        telemetry.addData("Robot Heading Offset", "(%.2f)", Math.toDegrees(driveHeadingOffset));
        telemetry.addData("Current Pose", "x(%.4f), y(%.4f), h(%.4f)",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true); // start teleop drive
        follower.update();
        runTime.resetTimer();
    }

    @Override
    public void loop() {
        if (runTime.getElapsedTimeSeconds() > 120) {
            intakeMotor.setIntakeOff();
            launcher.setLauncherOff();
            indicatorLight.setIndicatorColor(IndicatorLight.IndicatorColor.OFF);
            terminateOpModeNow();
        } else if (runTime.getElapsedTimeSeconds() > 113 && !isAlmostEndGame) {
            gamepad1.rumbleBlips(2); // gamepad rumble
            isAlmostEndGame = true;
        }

        // === Ball sensor and Indicator light ===
        boolean isBallDetected = leftBallSensor.isDetected() || rightBallSensor.isDetected();

        if (isBallDetected) {
            if (!indicateDetected) {
                indicateDetected = true;
                indicatorLight.setIndicatorColor(IndicatorLight.IndicatorColor.GREEN);
            }
        } else {
            if (indicateDetected) {
                indicateDetected = false;
                indicatorLight.setIndicatorColor(IndicatorLight.IndicatorColor.RED);
            }
        }

        // === Drive Control ===
        double forward = -gamepad1.left_stick_y * DRIVE_MAX_FORWARD_SPEED;
        double strafe  = -gamepad1.left_stick_x * DRIVE_MAX_FORWARD_SPEED;
        double rotate  = -gamepad1.right_stick_x * DRIVE_MAX_ANGULAR_SPEED;

        follower.setTeleOpDrive(forward, strafe, rotate, isRobotCentric, driveHeadingOffset);
        follower.update();

        for (int i = 0; i < motorPower.length; i++) {
            motorPower[i] = motors.get(i).getPower();
        }

        // === Intake Motor ===
        boolean intakeOn    = gamepad1.left_bumper;
        boolean intakeOff   = gamepad1.left_trigger > 0.5;
        boolean intakePanic = gamepad1.dpad_left;

        intakeMotor.run(intakeOn, intakeOff, intakePanic);

        // === Launcher ===
        boolean closeShot   = gamepad1.right_bumper;
        boolean farShot     = gamepad1.right_trigger > 0.5;
        boolean launchPanic = intakeMotor.isBusy() && isBallDetected &&
                                (sensorTime.getElapsedTimeSeconds() > FEEDER_PANIC_INTERVAL);

        if (launchPanic)
            sensorTime.resetTimer();
        launcher.launch(closeShot, farShot, launchPanic);

        // Log to telemetry for debugging
        telemetry.addData("Alliance Team", alliance.toString());
        telemetry.addData("Use Orientation", isRobotCentric ? "Robot centric" :
                isIMUResetRequested ? "Field Centric (Practice)" : "Field Centric");
        telemetry.addData("RunTime", "(%.4fs)", runTime.getElapsedTimeSeconds());
        telemetry.addData("Front Motor Power", "left (%.2f), right (%.2f)", motorPower[0],motorPower[2]);
        telemetry.addData("Back Motor Power", "left (%.2f), right (%.2f)", motorPower[1], motorPower[3]);
        telemetry.addData("Launcher State", launcher.getState());
        telemetry.addData("Launcher Speed", launcher.getLauncherVelocity());
        telemetry.addData("Sensor Timer", "(%.4fs)", sensorTime.getElapsedTimeSeconds());
        telemetry.addData("Intake State", intakeMotor.getState());
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.update();
    }
}