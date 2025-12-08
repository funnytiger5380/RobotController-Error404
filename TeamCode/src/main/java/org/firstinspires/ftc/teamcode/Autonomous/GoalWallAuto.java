package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Disabled
@Autonomous (name = "GoalWallAuto", group = "Error404")
public class GoalWallAuto extends LinearOpMode {
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

    private final double INTAKE_POWER = 0.75;
    private final double INTAKE_PANIC_TIME = 0.15;

    private final double ClOSE_LAUNCH_TARGET_VELOCITY = 1300;
    private final double CLOSE_LAUNCH_MIN_VELOCITY    = 1275;
    private final double FAR_LAUNCH_TARGET_VELOCITY   = 1610;
    private final double FAR_LAUNCH_MIN_VELOCITY      = 1600;

    private final double FEEDER_RUN_SECONDS = 0.10;
    private final double LAUNCH_COOLOFF_SECONDS = 0.50;

    // Timers
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime panicTimer = new ElapsedTime();

    // Autonomous step machine
    private enum StepState { IDLE, MOVE_BACK, LAUNCHING, FINISHED }
    private StepState stepState;

    // Alliance assignment
    private enum Alliance { BLUE, RED, NONE }
    private Alliance alliance = Alliance.NONE;

    private int shotCount = 0;

    @Override
    public void runOpMode() {
        /* === Drivetrain setup === */
        mecanumDrive.build(hardwareMap);

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

        while (opModeInInit()) {
            if (gamepad1.xWasPressed())
                alliance = Alliance.BLUE;
            else if (gamepad1.circleWasPressed())
                alliance = Alliance.RED;

            telemetry.addData("Status", "Initialized\n");
            telemetry.addData("Alliance", "Press Square/Circle button to select Blue/Red team\n");
            telemetry.addData("Alliance", alliance.toString());
            telemetry.update();
        }

        waitForStart();

        setStepState(StepState.IDLE);
        mecanumDrive.restartDrives();
        runtime.reset();

        while (opModeIsActive()) {
            switch (stepState) {
                case IDLE:
                    intakeMotor.setIntakeOn();
                    setStepState(StepState.MOVE_BACK);
                    break;

                case MOVE_BACK:
                    if (stepTimer.seconds() < 2.8) // move back 2.8s
                        mecanumDrive.runDrive(-0.3, 0, 0); // at 30% power
                    else {
                        mecanumDrive.stopDrives();
                        setStepState(StepState.LAUNCHING);
                    }
                    break;

                case LAUNCHING:
                    if (shotCount < 3) {
                        if (stepTimer.seconds() > 0.8) { // 0.8s second between launches
                            customLaunchCloseShot();  // intake panic + close shot
                            shotCount++;
                            setStepState(StepState.LAUNCHING);
                        }
                    }
                    else {
                        intakeMotor.setIntakeOff();
                        setStepState(StepState.FINISHED);
                    }
                    break;

                case FINISHED:
                    if (stepTimer.seconds() < 1.0) { // move to the side 1s
                        if (alliance == Alliance.BLUE)
                            mecanumDrive.runDrive(0, -0.5, 0); // at 50% power
                        else if (alliance == Alliance.RED)
                            mecanumDrive.runDrive(0, 0.5, 0); // at 50% power
                    } else
                        mecanumDrive.resetDrives();
                    break;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerFront(), mecanumDrive.getRightPowerFront());
            telemetry.addData("Back Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerBack(), mecanumDrive.getRightPowerBack());
            telemetry.addData("Launcher State", launcher.getState());
            telemetry.addData("Launcher Speed", launcher.getLauncherVelocity());
            telemetry.addData("Intake State", intakeMotor.getState());
            telemetry.addData("Intake Direction", intakeMotor.getMotorDirection());
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addData("Robot Heading", mecanumDrive.getDriveHeading(AngleUnit.DEGREES));
            telemetry.addData("Step State", stepState);
            telemetry.addData("Shot Count", shotCount);
            telemetry.addData("Panic Time", panicTimer.toString());
            telemetry.update();
        }
    }

    public void setStepState(StepState newStepState) {
        this.stepState = newStepState;
        stepTimer.reset();
    }

    private void customLaunchFarShot() {
        panicTimer.reset();
        intakeMotor.setIntakePanic(INTAKE_PANIC_TIME);
        launcher.launchFarShot();
    }

    private void customLaunchFarShot(int count) {
        for (int i = 0; i < count; i++)
            customLaunchFarShot();
    }

    private void customLaunchCloseShot() {
        panicTimer.reset();
        intakeMotor.setIntakePanic(INTAKE_PANIC_TIME);
        launcher.launchCloseShot();
    }

    private void customLaunchCloseShot(int count) {
        for (int i = 0; i < count; i++)
            customLaunchCloseShot();
    }
}
