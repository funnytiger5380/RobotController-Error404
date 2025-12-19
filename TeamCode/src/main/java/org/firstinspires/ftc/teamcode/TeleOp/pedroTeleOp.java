package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

@TeleOp (name = "pedroTeleOp", group = "Error404")
public class pedroTeleOp extends OpMode {
    private final double DRIVE_MAX_SPEED = 0.9;
    private final double INTAKE_POWER = 0.75;
    private final double INTAKE_PANIC_TIME = 0.1;

    private final double ClOSE_LAUNCH_TARGET_VELOCITY = 1300;
    private final double CLOSE_LAUNCH_MIN_VELOCITY    = 1275;
    private final double FAR_LAUNCH_TARGET_VELOCITY   = 1600;
    private final double FAR_LAUNCH_MIN_VELOCITY      = 1580;

    private final double FEEDER_RUN_SECONDS = 0.10;
    private final double LAUNCH_COOLOFF_SECONDS = 0.20;

    // === Drivetrain motors ===
    /* use Pedro pathing Mecanum drivetrain */
    private Mecanum mecanum;
    private List<DcMotorEx> motors;
    double [] motorPower;

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

    // === Run timer & Misc. ===
    private final ElapsedTime runtime = new ElapsedTime();
    private enum AllianceColor { BLUE, RED, NONE }
    private AllianceColor alliance = AllianceColor.NONE;

    private boolean isAlmostEndGame = false;
    private boolean isRobotCentric = true;
    private boolean isIMUResetRequested = false;
    private double driveHeadingOffset = 0.0;

    @Override
    public void init() {
        /* === Drivetrain setup === */
        follower = Constants.createFollower(hardwareMap);

        mecanum = (Mecanum) follower.getDrivetrain();
        mecanum.setMaxPowerScaling(DRIVE_MAX_SPEED);

        motors = mecanum.getMotors();
        motorPower = new double[motors.size()];

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
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed())
            alliance = AllianceColor.BLUE;
        else if (gamepad1.circleWasPressed())
            alliance = AllianceColor.RED;

        if (gamepad1.triangleWasPressed())
            isRobotCentric = false;
        else if (gamepad1.crossWasPressed())
            isRobotCentric = true;

        if (gamepad1.leftBumperWasPressed())
            isIMUResetRequested = true;
        else if (gamepad1.rightBumperWasPressed())
            isIMUResetRequested = false;

        if (!isRobotCentric) {
            if (isIMUResetRequested) {
                if (alliance == AllianceColor.BLUE)
                    driveHeadingOffset = -90.0;
                else if (alliance == AllianceColor.RED)
                    driveHeadingOffset = 90.0;
            }
        } else {
            driveHeadingOffset = 0.0;
        }

        telemetry.addData("Status", "Initialized\n");
        telemetry.addData("Alliance", "Press Square/Circle button to select Blue/Red team\n");
        telemetry.addData("Use Orientation", "Press Triangle/Cross button to choose Field/Robot centric Orientation\n");
        telemetry.addData("Request Field IMU reset?", "Press Left/Right Bumper to reset/not reset Field IMU\n");
        telemetry.addData("-----------------", "\n");
        telemetry.addData("Alliance Team", alliance.toString());
        telemetry.addData("Use Orientation", isRobotCentric ? "Robot centric" : "Field centric");
        telemetry.addData("Robot Heading Offset", driveHeadingOffset);
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:", AngleUnit.normalizeDegrees(Math.toDegrees(follower.getPose().getHeading())));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true);
        follower.update();
        runtime.reset();
    }

    @Override
    public void loop() {
        // === Drive Control ===
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, rotate, isRobotCentric, driveHeadingOffset);
        follower.update();

        for (int i = 0; i < motorPower.length; i++) {
            motorPower[i] = motors.get(i).getPower();
        }

        if (runtime.seconds() > 120.0) {
            terminateOpModeNow();
        } else if (runtime.seconds() > 110.0 && !isAlmostEndGame) {
            gamepad1.rumbleBlips(3);
            isAlmostEndGame = true;
        }

        // === Intake Motor ===
        boolean intakeOn  = gamepad1.left_bumper;
        boolean intakeOff = (gamepad1.left_trigger > 0.5);
        boolean panic     = gamepad1.dpad_left;

        intakeMotor.run(intakeOn, intakeOff, panic);

        // === Launcher ===
        boolean closeShotRequested = gamepad1.right_bumper;
        boolean farShotRequested   = (gamepad1.right_trigger > 0.5);

        launcher.launch(closeShotRequested, farShotRequested);

        // === Status Output ===
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Motor Power", "left (%.2f), right (%.2f)", motorPower[0],motorPower[2]);
        telemetry.addData("Back Motor Power", "left (%.2f), right (%.2f)", motorPower[1], motorPower[3]);
        telemetry.addData("Launcher State", launcher.getState());
        telemetry.addData("Launcher Speed", launcher.getLauncherVelocity());
        telemetry.addData("Intake State", intakeMotor.getState());
        telemetry.addData("Intake Direction", intakeMotor.getMotorDirection());
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:", AngleUnit.normalizeDegrees(Math.toDegrees(follower.getPose().getHeading())));
        telemetry.update();
    }
}