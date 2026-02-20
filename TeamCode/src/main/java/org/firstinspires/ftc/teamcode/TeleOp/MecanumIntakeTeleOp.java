package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.DigitalSensor;
import org.firstinspires.ftc.teamcode.mechanisms.IndicatorLight;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name = "MecanumIntakeTeleOp", group = "Error404")
public class MecanumIntakeTeleOp extends OpMode {

    double DRIVE_MAX_POWER = 1.0;
    double DRIVE_MAX_FORWARD_SPEED = 1.0;
    double DRIVE_MAX_ANGULAR_SPEED = 0.7;

    double INTAKE_POWER = 0.75;
    double INTAKE_PANIC_TIME = 0.10;

    double SUPER_CLOSE_LAUNCH_TARGET_VELOCITY = 1200;
    double SUPER_CLOSE_LAUNCH_MIN_VELOCITY    = 1190;
    double CLOSE_LAUNCH_TARGET_VELOCITY = 1300;
    double CLOSE_LAUNCH_MIN_VELOCITY    = 1290;
    double FAR_LAUNCH_TARGET_VELOCITY   = 1600;
    double FAR_LAUNCH_MIN_VELOCITY      = 1590;

    double FEEDER_RUN_SECONDS = 0.15;
    double FEEDER_PANIC_SECONDS = 0.10;
    double FEEDER_PANIC_INTERVAL = FEEDER_PANIC_SECONDS + 0.10;
    double LAUNCH_COOL_OFF_SECONDS = 0.20;
    double LAUNCH_ON_SECOND_AT_IDLE = 2.0;

    private final ElapsedTime intakePanicTimer = new ElapsedTime();
    private boolean intakePanicActive = false;

    private final MecanumDrive mecanumDrive = new MecanumDrive()
            .leftFrontName("left_drive_front")
            .leftBackName("left_drive_back")
            .rightFrontName("right_drive_front")
            .rightBackName("right_drive_back")
            .leftFrontDirection(REVERSE)
            .leftBackDirection(REVERSE)
            .rightFrontDirection(FORWARD)
            .rightBackDirection(FORWARD)
            .useBrakeMode(true);

    private final IntakeMotor intakeMotor = new IntakeMotor()
            .motorName("intake")
            .motorUseBrakeMode(true)
            .motorDirection(REVERSE);

    private final Launcher launcher = new Launcher()
            .launcherName("launcher")
            .launcherUseBrakeMode(true)
            .launcherDirection(REVERSE)
            .leftFeederName("left_feeder")
            .rightFeederName("right_feeder")
            .leftFeederDirection(FORWARD)
            .rightFeederDirection(REVERSE);

    private final IndicatorLight indicatorLight = new IndicatorLight()
            .indicatorName("indicator_light");

    private boolean indicateDetected = false;

    private final DigitalSensor leftBallSensor = new DigitalSensor()
            .sensorName("left_ball_sensor")
            .sensorMode(DigitalChannel.Mode.INPUT);

    private final DigitalSensor rightBallSensor = new DigitalSensor()
            .sensorName("right_ball_sensor")
            .sensorMode(DigitalChannel.Mode.INPUT);

    private final ElapsedTime sensorTime = new ElapsedTime();
    private final ElapsedTime runTime = new ElapsedTime();

    private enum Alliance { BLUE, RED, NONE }
    private Alliance alliance = Alliance.NONE;

    private boolean isAlmostEndGame = false;
    private boolean isIMURequested = false;
    private double driveHeadingOffset = 0.0;

    @Override
    public void init() {

        mecanumDrive.build(hardwareMap);
        mecanumDrive.setMaxPower(DRIVE_MAX_POWER);

        mecanumDrive.initRevIMU(
                hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        intakeMotor.build(hardwareMap);
        intakeMotor.setPower(INTAKE_POWER);
        intakeMotor.setPanicTime(INTAKE_PANIC_TIME);

        launcher.build(hardwareMap);
        launcher.launcherOnAtIdle();
        launcher.setLauncherReadyVelocity(SUPER_CLOSE_LAUNCH_TARGET_VELOCITY);
        launcher.setLauncherCloseVelocity(CLOSE_LAUNCH_TARGET_VELOCITY, CLOSE_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherFarVelocity(FAR_LAUNCH_TARGET_VELOCITY, FAR_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherCoolOffSec(LAUNCH_COOL_OFF_SECONDS);
        launcher.setLauncherOnSecAtIdle(LAUNCH_ON_SECOND_AT_IDLE);
        launcher.setFeederRunSec(FEEDER_RUN_SECONDS);
        launcher.setPanicRunSec(FEEDER_PANIC_SECONDS);

        indicatorLight.build(hardwareMap);

        leftBallSensor.build(hardwareMap);
        rightBallSensor.build(hardwareMap);
    }

    @Override
    public void init_loop() {

        if (gamepad1.xWasPressed())
            alliance = Alliance.BLUE;
        else if (gamepad1.circleWasPressed())
            alliance = Alliance.RED;

        if (gamepad1.triangleWasPressed())
            isIMURequested = true;
        else if (gamepad1.crossWasPressed())
            isIMURequested = false;

        if (isIMURequested) {
            if (alliance == Alliance.BLUE)
                driveHeadingOffset = -90.0;
            else if (alliance == Alliance.RED)
                driveHeadingOffset = 90.0;
        } else {
            driveHeadingOffset = 0.0;
        }

        telemetry.addData("Alliance Team", alliance.toString());
        telemetry.addData("Use Orientation",
                isIMURequested ? "Field centric" : "Robot centric");
        telemetry.addData("Robot Heading Offset", driveHeadingOffset);
        telemetry.update();
    }

    @Override
    public void start() {

        if (!isIMURequested)
            mecanumDrive.resetDriveYaw();

        mecanumDrive.setDriveAngularOffset(driveHeadingOffset);
        mecanumDrive.restartDrives();

        runTime.reset();
        sensorTime.reset();
    }

    @Override
    public void loop() {

        if (runTime.seconds() > 120.0) {
            intakeMotor.setIntakeOff();
            launcher.setLauncherOff();
            indicatorLight.setIndicatorColor(
                    IndicatorLight.IndicatorColor.OFF
            );
            terminateOpModeNow();
        }

        double forward = -gamepad1.left_stick_y * DRIVE_MAX_FORWARD_SPEED;
        double strafe  = gamepad1.left_stick_x * DRIVE_MAX_FORWARD_SPEED;
        double rotate  = gamepad1.right_stick_x * DRIVE_MAX_ANGULAR_SPEED;

        if (!isIMURequested)
            mecanumDrive.runDrive(forward, strafe, rotate);
        else
            mecanumDrive.runDriveFieldRelative(forward, strafe, rotate);

        boolean intakeOn = gamepad1.left_bumper;
        boolean intakeOff = gamepad1.left_trigger > 0.5;
        boolean panicButton = gamepad1.dpad_left;

        if (panicButton && !intakePanicActive) {
            intakePanicActive = true;
            intakePanicTimer.reset();
        }

        if (intakePanicActive) {
            intakeMotor.setIntakePanic();

            if (intakePanicTimer.seconds() >= INTAKE_PANIC_TIME)
                intakePanicActive = false;
        } else {
            intakeMotor.run(intakeOn, intakeOff, false);
        }

        boolean isBallDetected =
                leftBallSensor.isDetected() ||
                        rightBallSensor.isDetected();

        if (isBallDetected) {
            if (!indicateDetected) {
                indicateDetected = true;
                indicatorLight.setIndicatorColor(
                        IndicatorLight.IndicatorColor.GREEN
                );
            }
        } else {
            if (indicateDetected) {
                indicateDetected = false;
                indicatorLight.setIndicatorColor(
                        IndicatorLight.IndicatorColor.RED
                );
            }
        }

        // === Launcher ===
        boolean superCloseShot  = gamepad1.circleWasPressed();
        boolean normalCloseShot = gamepad1.right_bumper;
        boolean closeShot   = superCloseShot || normalCloseShot;
        boolean farShot     = gamepad1.right_trigger > 0.5;
        boolean launchReady = gamepad1.left_bumper || gamepad1.left_trigger > 0.5 ||
                                gamepad2.left_bumper || gamepad2.left_trigger > 0.5;
        boolean launchPanic = intakeMotor.isBusy() && (sensorTime.seconds() > FEEDER_PANIC_INTERVAL);

        if (launchPanic)
            sensorTime.reset();

        if (superCloseShot)
            launcher.setLauncherCloseVelocity(SUPER_CLOSE_LAUNCH_TARGET_VELOCITY, SUPER_CLOSE_LAUNCH_MIN_VELOCITY);
        else if (normalCloseShot)
            launcher.setLauncherCloseVelocity(CLOSE_LAUNCH_TARGET_VELOCITY, CLOSE_LAUNCH_MIN_VELOCITY);

        launcher.launch(closeShot, farShot, launchReady, launchPanic);

        telemetry.addData("Intake State", intakeMotor.getState());
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.update();
    }
}
