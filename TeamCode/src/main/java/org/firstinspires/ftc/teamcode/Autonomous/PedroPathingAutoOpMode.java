package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.mechanisms.DigitalSensor;
import org.firstinspires.ftc.teamcode.mechanisms.FollowerAction;
import org.firstinspires.ftc.teamcode.mechanisms.FollowerPathBuilder;
import org.firstinspires.ftc.teamcode.mechanisms.FollowerPose;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.SelectableFollowerAction;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Function;

public class PedroPathingAutoOpMode extends OpMode {
    Follower follower;
    FollowerPose followerPose;
    Pose currentPose = new Pose();
    boolean useRedPose, useFarStartPose, useFarStopPose;

    enum PathState {
        START_POSE, SCORE_POSE, HIGH_SPIKE_POSE, GRAB_HIGH_SPIKE,
        MID_SPIKE_POSE, GRAB_MID_SPIKE, MID_SPIKE_RETURN_POSE,
        LOW_SPIKE_POSE, GRAB_LOW_SPIKE, LOW_SPIKE_RETURN_POSE,
        GATE_OPEN_UP_POSE, GATE_CONTACT_UP_POSE,
        GATE_OPEN_DOWN_POSE, GATE_CONTACT_DOWN_POSE,
        GATE_RETURN_POSE, STOP_POSE
    } PathState pathState;
    FollowerPathBuilder pathBuilder;
    boolean isHighSpikeGrabbed = false;
    boolean isMidSpikeGrabbed = false;

    FollowerAction nextAction = null;
    SelectableFollowerAction followerSelectedAction = new SelectableFollowerAction(
            "<< Follower Action Sequence Selection >>", a -> {
        a.add("CLOSE_LAUNCH", FollowerAction.CLOSE_LAUNCH);
        a.add("FAR_LAUNCH", FollowerAction.FAR_LAUNCH);
        a.add("GRAB_HIGH_SPIKE", FollowerAction.GRAB_HIGH_SPIKE);
        a.add("GRAB_MID_SPIKE", FollowerAction.GRAB_MID_SPIKE);
        a.add("GRAB_LOW_SPIKE", FollowerAction.GRAB_LOW_SPIKE);
        a.add("OPEN_GATE", FollowerAction.OPEN_GATE);
        a.add("STOP", FollowerAction.STOP);
    });

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

    // Sensor to detect whether the artifact is present for launching
    DigitalSensor ballSensor = new DigitalSensor()
            .sensorName("ball_sensor")
            .sensorMode(DigitalChannel.Mode.INPUT);

    // Drivetrain constants
    double PATH_SPEED = 0.75;
    double GRAB_SPEED = 0.35;
    double GATE_SPEED = 0.35;

    // Intake motor constants
    double INTAKE_POWER = 0.75;
    double INTAKE_PANIC_TIME = 0.15;

    // Launcher constants
    double ClOSE_LAUNCH_TARGET_VELOCITY = 1300;
    double CLOSE_LAUNCH_MIN_VELOCITY = 1280;
    double CLOSE_LAUNCH_INTERVAL_SECONDS = 0.25;

    double FAR_LAUNCH_TARGET_VELOCITY = 1600;
    double FAR_LAUNCH_MIN_VELOCITY = 1580;
    double FAR_LAUNCH_INTERVAL_SECONDS = 0.75;

    double FEEDER_RUN_SECONDS = 0.10;
    double LAUNCH_COOL_OFF_SECONDS = 0.20;

    // OpMode timers
    Timer runTime = new Timer();
    Timer pathTimer = new Timer();
    Timer launchTimer = new Timer();

    @Override
    public void init() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        followerPose = useRedPose ? new FollowerPose("red") : new FollowerPose("blue");
        pathBuilder = new FollowerPathBuilder(follower, followerPose);

        if (useFarStartPose)
            followerPose.useFarStartPose();
        else
            followerPose.useCloseStartPose();

        if (useFarStopPose)
            followerPose.useFarStopPose();
        else
            followerPose.useCloseStopPose();

        follower.setStartingPose(followerPose.startPose);
        follower.update();

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

        // Initialize Digital sensor
        ballSensor.build(hardwareMap);
    }

    @Override
    public void init_loop() {
        // select follower action sequence
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed())
            followerSelectedAction.decrementSelected();
        else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed())
            followerSelectedAction.incrementSelected();
        else if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed())
            followerSelectedAction.addSelected();
        else if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed())
            followerSelectedAction.removeLast();

        for (String line : followerSelectedAction.getSelectableLines())
            telemetry.addLine(line);

        telemetry.addLine();
        telemetry.addLine("Follower action sequence:");
        telemetry.addLine(followerSelectedAction.getActionLines());
        telemetry.addLine("------------");
        telemetry.addData("OpMode", useFarStartPose ? useRedPose ? "RedFarStartAuto" : "BlueFarStartAuto"
                : useRedPose ? "RedCloseStartAuto" : "BlueCloseStartAuto");
        telemetry.addData("Start Pose", "x(%.1f), y(%.1f), h(%.1f)",
                followerPose.startPose.getX(), followerPose.startPose.getY(), Math.toDegrees(followerPose.startPose.getHeading()));
        telemetry.addData("Stop Pose", "x(%.1f), y(%.1f), h(%.1f)",
                followerPose.stopPose.getX(), followerPose.stopPose.getY(), Math.toDegrees(followerPose.stopPose.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        runTime.resetTimer();
        setPathState(PathState.START_POSE);
        getNextAction();
    }

    @Override
    public void loop() {
        if (runTime.getElapsedTimeSeconds() > 30) // stop after 30 seconds
            terminateOpModeNow();
        else
            updatePathState(); // Follower drives through the built path                                                                                                  e given pathing

        // Update follower pathing on every iteration
        follower.update();
        currentPose = follower.getPose(); // the follower current pose

        // Log to telemetry for debugging
        telemetry.addData("OpMode", useFarStartPose ? useRedPose ? "RedFarStartAuto" : "BlueFarStartAuto"
                : useRedPose ? "RedCloseStartAuto" : "BlueCloseStartAuto");
        telemetry.addData("RunTime", "(%.4fs)", runTime.getElapsedTimeSeconds());
        telemetry.addData("Current Pose", "x(%.4f), y(%.4f), h(%.4f)",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Current Action", isNextAction(null) ? "None": nextAction.toString());
        telemetry.addLine("Follower action sequence remaining:");
        telemetry.addLine(followerSelectedAction.getActionLines());
        telemetry.addData("PathState", pathState.toString());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    void setPathState(PathState newPathState) {
        this.pathState = newPathState;
        pathTimer.resetTimer();
    }

    void updatePathState() {
        if (isNextAction(null)) {
            intakeMotor.setIntakeOff();
            launcher.setLauncherOff();
            follower.holdPoint(currentPose);
        } else {
            switch (pathState) {
                case START_POSE:
                    if (isNextAction(FollowerAction.CLOSE_LAUNCH)) {
                        intakeMotor.setIntakeOn();
                        followerPose.useCloseScorePose();
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2Score);
                        setPathState(PathState.SCORE_POSE);
                    } else if (isNextAction(FollowerAction.FAR_LAUNCH)) {
                        intakeMotor.setIntakeOn();
                        followerPose.useFarScorePose();
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2Score);
                        setPathState(PathState.SCORE_POSE);
                    } else if (isNextAction(FollowerAction.STOP)) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2Stop);
                        setPathState(PathState.STOP_POSE);
                    } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2Gate);
                        setPathState(PathState.GATE_OPEN_UP_POSE);
                    } else if (isNextAction(FollowerAction.GRAB_HIGH_SPIKE)) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2HighSpk);
                        setPathState(PathState.HIGH_SPIKE_POSE);
                    } else if (isNextAction(FollowerAction.GRAB_MID_SPIKE)) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2MidSpk);
                        setPathState(PathState.MID_SPIKE_POSE);
                    } else if (isNextAction(FollowerAction.GRAB_LOW_SPIKE)) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2LowSpk);
                        setPathState(PathState.LOW_SPIKE_POSE);
                    }
                    break;

                case SCORE_POSE:
                    if (!follower.isBusy()) { // wait till follower path update is done
                        if (isNextAction(FollowerAction.CLOSE_LAUNCH))
                            customLaunchCloseShot(3, CLOSE_LAUNCH_INTERVAL_SECONDS);
                        else if (isNextAction(FollowerAction.FAR_LAUNCH))
                            customLaunchFarShot(3, FAR_LAUNCH_INTERVAL_SECONDS);

                        intakeMotor.setIntakeOff();
                        getNextAction(); // get the next follower action

                        if (isNextAction(FollowerAction.STOP)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_scorePos2Stop);
                            setPathState(PathState.STOP_POSE);
                        } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_scorePos2Gate);
                            setPathState(PathState.GATE_OPEN_UP_POSE);
                        } else if (isNextAction(FollowerAction.GRAB_HIGH_SPIKE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_scorePos2HighSpk);
                            setPathState(PathState.HIGH_SPIKE_POSE);
                        } else if (isNextAction(FollowerAction.GRAB_MID_SPIKE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_scorePos2MidSpk);
                            setPathState(PathState.MID_SPIKE_POSE);
                        } else if (isNextAction(FollowerAction.GRAB_LOW_SPIKE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_scorePos2LowSpk);
                            setPathState(PathState.LOW_SPIKE_POSE);
                        }
                    }
                    break;

                case HIGH_SPIKE_POSE:
                    if (!follower.isBusy()) {
                        intakeMotor.setIntakeOn();
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkPos2GrabEnd, GRAB_SPEED);
                        setPathState(PathState.GRAB_HIGH_SPIKE);
                    }
                    break;

                case MID_SPIKE_POSE:
                    if (!follower.isBusy()) {
                        intakeMotor.setIntakeOn();
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkPos2GrabEnd, GRAB_SPEED);
                        setPathState(PathState.GRAB_MID_SPIKE);
                    }
                    break;

                case LOW_SPIKE_POSE:
                    if (!follower.isBusy()) {
                        intakeMotor.setIntakeOn();
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkPos2GrabEnd, GRAB_SPEED);
                        setPathState(PathState.GRAB_LOW_SPIKE);
                    }
                    break;

                case GRAB_HIGH_SPIKE:
                    if (!follower.isBusy()) {
                        isHighSpikeGrabbed = true;
                        getNextAction(); // get the next follower action

                        if (isNextAction(FollowerAction.CLOSE_LAUNCH)) {
                            followerPose.useCloseScorePose();
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkEnd2Score);
                            setPathState(PathState.SCORE_POSE);
                        } else if (isNextAction(FollowerAction.FAR_LAUNCH)) {
                            followerPose.useFarScorePose();
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkEnd2Score);
                            setPathState(PathState.SCORE_POSE);
                        } else if (isNextAction(FollowerAction.STOP)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkEnd2Stop);
                            setPathState(PathState.STOP_POSE);
                        } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkEnd2Gate);
                            setPathState(PathState.GATE_OPEN_UP_POSE);
                        }
                    }
                    break;

                case GRAB_MID_SPIKE:
                    if (!follower.isBusy()) {
                        isMidSpikeGrabbed = true;
                        getNextAction(); // get the next follower action

                        if (isNextAction(FollowerAction.CLOSE_LAUNCH)) {
                            followerPose.useCloseScorePose();
                            if (isHighSpikeGrabbed) {
                                followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkEnd2Score);
                                setPathState(PathState.SCORE_POSE);
                            } else {
                                followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkEnd2Return);
                                setPathState(PathState.MID_SPIKE_RETURN_POSE);
                            }
                        } else if (isNextAction(FollowerAction.FAR_LAUNCH)) {
                            followerPose.useFarScorePose();
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkEnd2Score);
                            setPathState(PathState.SCORE_POSE);
                        } else if (isNextAction(FollowerAction.STOP)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkEnd2Stop);
                            setPathState(PathState.STOP_POSE);
                        } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkEnd2Gate);
                            setPathState(PathState.GATE_OPEN_DOWN_POSE);
                        }
                    }
                    break;

                case MID_SPIKE_RETURN_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkReturn2Score);
                        setPathState(PathState.SCORE_POSE);
                    }
                    break;

                case GRAB_LOW_SPIKE:
                    if (!follower.isBusy()) {
                        getNextAction(); // get the next follower action

                        if (isNextAction(FollowerAction.CLOSE_LAUNCH)) {
                            followerPose.useCloseScorePose();
                            if (isMidSpikeGrabbed) {
                                followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkEnd2Score);
                                setPathState(PathState.SCORE_POSE);
                            } else {
                                followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkEnd2Return);
                                setPathState(PathState.LOW_SPIKE_RETURN_POSE);
                            }
                        } else if (isNextAction(FollowerAction.FAR_LAUNCH)) {
                            followerPose.useFarScorePose();
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkEnd2Score);
                            setPathState(PathState.SCORE_POSE);
                        } else if (isNextAction(FollowerAction.STOP)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkEnd2Stop);
                            setPathState(PathState.STOP_POSE);
                        } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkEnd2Gate);
                            setPathState(PathState.GATE_OPEN_DOWN_POSE);
                        }
                    }
                    break;

                case LOW_SPIKE_RETURN_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkReturn2Score);
                        setPathState(PathState.SCORE_POSE);
                    }
                    break;

                case GATE_OPEN_UP_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gatePosUp2Contact, GATE_SPEED);
                        setPathState(PathState.GATE_CONTACT_UP_POSE);
                    }
                    break;

                case GATE_OPEN_DOWN_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gatePosDw2Contact, GATE_SPEED);
                        setPathState(PathState.GATE_CONTACT_DOWN_POSE);
                    }
                    break;

                case GATE_CONTACT_UP_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gateContactUp2Return);
                        setPathState(PathState.GATE_RETURN_POSE);
                    }
                    break;

                case GATE_CONTACT_DOWN_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gateContactDw2Return);
                        setPathState(PathState.GATE_RETURN_POSE);
                    }
                    break;

                case GATE_RETURN_POSE:
                    if (!follower.isBusy()) {
                        getNextAction();

                        if (isNextAction(FollowerAction.CLOSE_LAUNCH)) {
                            followerPose.useCloseScorePose();
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gateReturn2Score);
                            setPathState(PathState.SCORE_POSE);
                        } else if (isNextAction(FollowerAction.FAR_LAUNCH)) {
                            followerPose.useFarScorePose();
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gateReturn2Score);
                            setPathState(PathState.SCORE_POSE);
                        } else if (isNextAction(FollowerAction.STOP)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gateReturnPos2Stop);
                            setPathState(PathState.STOP_POSE);
                        } else if (isNextAction(FollowerAction.GRAB_HIGH_SPIKE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gateReturnPos2HighSpk);
                            setPathState(PathState.HIGH_SPIKE_POSE);
                        } else if (isNextAction(FollowerAction.GRAB_MID_SPIKE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gateReturnPos2MidSpk);
                            setPathState(PathState.MID_SPIKE_POSE);
                        } else if (isNextAction(FollowerAction.GRAB_LOW_SPIKE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gateReturnPos2LowSpk);
                            setPathState(PathState.LOW_SPIKE_POSE);
                        }
                    }
                    break;

                case STOP_POSE:
                    if (!follower.isBusy()) {
                        intakeMotor.setIntakeOff();
                        follower.holdPoint(currentPose);
                    }
                    break;
            }
        }
    }

    void getNextAction() {
        nextAction = followerSelectedAction.getNextAction();
    }

    boolean isNextAction(FollowerAction action) {
        return nextAction == action;
    }

    void followingPath(FollowerPathBuilder pb, Function<FollowerPathBuilder, PathChain> buildPath,
                       double pathSpeed) {
        follower.followPath(buildPath.apply(pb), pathSpeed, true);
    }

    void followingPath(FollowerPathBuilder pb, Function<FollowerPathBuilder, PathChain> buildPath) {
        followingPath(pb, buildPath, PATH_SPEED);
    }

    void customLaunchCloseShot() {
        if (!ballSensor.isDetected())
            intakeMotor.setIntakePanic(INTAKE_PANIC_TIME);
        launcher.launchCloseShot();
    }

    void customLaunchCloseShot(int count, double interval) {
        launcher.launcherOnAtIdle();
        for (int i = 0; i < count; i++) {
            launchTimer.resetTimer();
            do {} while (launchTimer.getElapsedTimeSeconds() < interval);
            customLaunchCloseShot();
        }
        launcher.launcherOffAtIdle();
        if (ballSensor.isDetected()) // additional last check
            launcher.launchCloseShot();
        else
            launcher.setLauncherOff();
    }

    void customLaunchFarShot() {
        if (!ballSensor.isDetected())
            intakeMotor.setIntakePanic(INTAKE_PANIC_TIME);
        launcher.launchFarShot();
    }

    void customLaunchFarShot(int count, double interval) {
        launcher.launcherOnAtIdle();
        for (int i = 0; i < count; i++) {
            launchTimer.resetTimer();
            do {} while (launchTimer.getElapsedTimeSeconds() < interval);
            customLaunchFarShot();
        }
        launcher.launcherOffAtIdle();
        if (ballSensor.isDetected()) // additional last check
            launcher.launchFarShot();
        else
            launcher.setLauncherOff();
    }
}