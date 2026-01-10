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
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * TeleOp program for an FTC robot with:
 *  - Four-motor Mecanum wheel drivetrain
 *  - One launcher motor using velocity control
 *  - Two feeder CRServos
 *  - Intake motor
 *
 * CONTROLS:
 *  Left stick Y = forward/backward (up = forward, down = backward)
 *  Left stick X = strafe (left = left, right = right)
 *  Right stick X = rotate (left = CCW, right = CW)
 *
 *  Right bumper = launcher to fire one close shot
 *  Right trigger = launcher to fire one far shot
 *
 *  Left bumper  = intake forward
 *  Left trigger = intake stop
 *  D-pad Left   = intake reverse
 */

@TeleOp (name = "MecanumIntakeTeleOp", group = "Error404")
public class MecanumIntakeTeleOp extends OpMode {
    private final double DRIVE_MAX_POWER = 1.0;
    private final double DRIVE_MAX_FORWARD_SPEED = 1.0;
    private final double DRIVE_MAX_ANGULAR_SPEED = 0.7;

    private final double INTAKE_POWER = 0.75;
    private final double INTAKE_PANIC_TIME = 0.05;

    private final double ClOSE_LAUNCH_TARGET_VELOCITY = 1300;
    private final double CLOSE_LAUNCH_MIN_VELOCITY    = 1280;
    private final double FAR_LAUNCH_TARGET_VELOCITY   = 1600;
    private final double FAR_LAUNCH_MIN_VELOCITY      = 1580;

    private final double FEEDER_RUN_SECONDS = 0.10;
    private final double FEEDER_PANIC_INTERVAL = 0.10 + FEEDER_RUN_SECONDS;
    private final double LAUNCH_COOLOFF_SECONDS = 0.20;

    // === Drivetrain motors ===
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

    // === Intake ===
    private final IntakeMotor intakeMotor = new IntakeMotor()
            .motorName("intake")
            .motorUseBrakeMode(true)
            .motorDirection(REVERSE);

    // === Launcher and feeders ===
    private final Launcher launcher = new Launcher()
            .launcherName("launcher")
            .launcherUseBrakeMode(true)
            .launcherDirection(REVERSE)
            .leftFeederName("left_feeder")
            .rightFeederName("right_feeder")
            .leftFeederDirection(FORWARD)
            .rightFeederDirection(REVERSE);

    // === Ball sensor ===
    private final DigitalSensor ballSensor = new DigitalSensor()
            .sensorName("ball_sensor")
            .sensorMode(DigitalChannel.Mode.INPUT);
    private final ElapsedTime sensorTime = new ElapsedTime();

    // === Run timer & Misc. ===
    private final ElapsedTime runTime = new ElapsedTime();
    private enum Alliance { BLUE, RED, NONE }
    private Alliance alliance = Alliance.NONE;
    private boolean isAlmostEndGame = false;
    private boolean isIMURequested = false;
    private double driveHeadingOffset = 0.0;

    @Override
    public void init() {
        /* === Drivetrain setup === */
        mecanumDrive.build(hardwareMap);
        mecanumDrive.setMaxPower(DRIVE_MAX_POWER);

        /* === IMU setup === */
        mecanumDrive.initRevIMU(hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        /* === Intake setup === */
        intakeMotor.build(hardwareMap);
        intakeMotor.setPower(INTAKE_POWER);
        intakeMotor.setPanicTime(INTAKE_PANIC_TIME);

        /* === Launcher setup === */
        launcher.build(hardwareMap);
        launcher.setLauncherCloseVelocity(ClOSE_LAUNCH_TARGET_VELOCITY, CLOSE_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherFarVelocity(FAR_LAUNCH_TARGET_VELOCITY, FAR_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherCoolOffSec(LAUNCH_COOLOFF_SECONDS);
        launcher.setFeederRunSec(FEEDER_RUN_SECONDS);

        /* === Digital sensor === */
        ballSensor.build(hardwareMap);
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

        telemetry.addData("Alliance", "Press Square/Circle button to select Blue/Red team\n");
        telemetry.addData("Use RevIMU", "Press Triangle/Cross button to choose Field/Robot Orientation\n");
        telemetry.addLine();
        telemetry.addLine("------------");
        telemetry.addData("Alliance Team", alliance.toString());
        telemetry.addData("Use Orientation", isIMURequested ? "Field centric" : "Robot centric");
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
        // === Drive Control ===
        double forward = -gamepad1.left_stick_y * DRIVE_MAX_FORWARD_SPEED;
        double strafe  = gamepad1.left_stick_x  * DRIVE_MAX_FORWARD_SPEED;
        double rotate  = gamepad1.right_stick_x * DRIVE_MAX_ANGULAR_SPEED;

        if (runTime.seconds() > 120.0) {
            intakeMotor.setIntakeOff();
            launcher.setLauncherOff();
            terminateOpModeNow();
        } else if (runTime.seconds() > 112.0 && !isAlmostEndGame) {
            gamepad1.rumbleBlips(2);
            isAlmostEndGame = true;
        }

        if (!isIMURequested)
            mecanumDrive.runDrive(forward, strafe, rotate);
        else
            mecanumDrive.runDriveFieldRelative(forward, strafe, rotate);

        // === Intake Motor ===
        boolean intakeOn    = gamepad1.left_bumper;
        boolean intakeOff   = gamepad1.left_trigger > 0.5;
        boolean intakePanic = gamepad1.dpad_left;

        intakeMotor.run(intakeOn, intakeOff, intakePanic);

        // === Launcher ===
        boolean closeShot   = gamepad1.right_bumper;
        boolean farShot     = gamepad1.right_trigger > 0.5;
        boolean launchPanic = intakeMotor.isBusy() && (sensorTime.seconds() > FEEDER_PANIC_INTERVAL);

        if (launchPanic)
            sensorTime.reset();
        launcher.launch(closeShot, farShot, launchPanic);

        // === Status Output ===
        telemetry.addData("Alliance Team", alliance.toString());
        telemetry.addData("Use Orientation", isIMURequested ? "Field centric" : "Robot centric");
        telemetry.addData("Run Time", runTime.toString());
        telemetry.addData("Front Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerFront(), mecanumDrive.getRightPowerFront());
        telemetry.addData("Back Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerBack(), mecanumDrive.getRightPowerBack());
        telemetry.addData("Launcher State", launcher.getState());
        telemetry.addData("Launcher Speed", launcher.getLauncherVelocity());
        telemetry.addData("Intake State", intakeMotor.getState());
        telemetry.addData("Intake Direction", intakeMotor.getMotorDirection());
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Robot Heading", mecanumDrive.getDriveHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}