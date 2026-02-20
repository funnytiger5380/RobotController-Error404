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
import org.firstinspires.ftc.teamcode.mechanisms.IndicatorLight;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.SelectableFollowerAction;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Function;

public class PedroPathingAutoOpMode extends OpMode {
    Follower follower;
    FollowerPose followerPose;
    Pose currentPose = new Pose();

    // Configurable OpMode parameters
    boolean useRedPose, useFarStartPose, useFarStopPose;

    enum PathState {
        START_POSE, SCORE_POSE, STOP_POSE,
        HIGH_SPIKE_POSE, GRAB_HIGH_SPIKE, HIGH_SPIKE_RETURN_POSE,
        MID_SPIKE_POSE, GRAB_MID_SPIKE, MID_SPIKE_RETURN_POSE,
        LOW_SPIKE_POSE, GRAB_LOW_SPIKE, LOW_SPIKE_RETURN_POSE,
        GATE_OPEN_POSE, GATE_CONTACT_POSE, GATE_RETURN_POSE
    } PathState pathState;
    FollowerPathBuilder pathBuilder;
    boolean isHighSpikeGrabbed = false;
    boolean isMidSpikeGrabbed = false;
    boolean isLowSpikeGrabbed = false;

    // Select the follower action sequence
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
    DigitalSensor leftBallSensor = new DigitalSensor()
            .sensorName("left_ball_sensor")
            .sensorMode(DigitalChannel.Mode.INPUT);
    DigitalSensor rightBallSensor = new DigitalSensor()
            .sensorName("right_ball_sensor")
            .sensorMode(DigitalChannel.Mode.INPUT);
    boolean isBallDetected = false;

    // Indicator light to signal the artifact status
    IndicatorLight indicatorLight = new IndicatorLight()
            .indicatorName("indicator_light");
    boolean indicateDetected = false;

    // Drivetrain constants
    double PATH_SPEED_NORMAL = 0.90;
    double PATH_SPEED_SLOW = 0.70;
    double GRAB_SPEED = 0.40;
    double GATE_SPEED = 0.40;

    // Intake motor constants
    double INTAKE_POWER = 0.75;
    double INTAKE_PANIC_TIME = 0.10;
    double INTAKE_PANIC_WAIT = 0.30;

    // Launcher constants
    double CLOSE_LAUNCH_TARGET_VELOCITY = 1300;
    double CLOSE_LAUNCH_MIN_VELOCITY = 1290;
    double CLOSE_LAUNCH_INTERVAL_SECONDS = 0.20;

    double FAR_LAUNCH_TARGET_VELOCITY = 1600;
    double FAR_LAUNCH_MIN_VELOCITY = 1590;
    double FAR_LAUNCH_INTERVAL_SECONDS = 0.30;

    double FEEDER_RUN_SECONDS = 0.15;
    double FEEDER_PANIC_SECONDS = 0.10;
    double FEEDER_PANIC_INTERVAL = FEEDER_PANIC_SECONDS + 0.10;
    double LAUNCH_COOL_OFF_SECONDS = 0.20;
    double LAUNCH_ON_SECOND_AT_IDLE = 3.3;

    // OpMode timers
    Timer runTime = new Timer();
    Timer launchTimer = new Timer();
    Timer sensorTimer = new Timer();

    // Debug variables
    int panicCount = 0;
    int launchCount = 0;

    @Override
    public void init() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        followerPose = useRedPose ? new FollowerPose("red") : new FollowerPose("blue");
        pathBuilder = new FollowerPathBuilder(follower, followerPose);

        if (useFarStartPose) { // use far start pose
            followerPose.useFarStartPose();
            if (useRedPose) { // red far
                followerPose.setMidSpkPose(90.0, 63.0, Math.toRadians(-12.0));
                followerPose.setMidSpkEnd(123.0, 61.0, Math.toRadians(-17.0));
                followerPose.setStopPose(100.0, 75.0, Math.toRadians(-10.0));
            }
            else { // blue far
                followerPose.setMidSpkPose(44.0, 63.0, Math.toRadians(180.0));
                followerPose.setMidSpkEnd(19.0, 63.0, Math.toRadians(180.0));
            }
        } else { // use close start pose
            followerPose.useCloseStartPose();
            if (useRedPose) { // red close
                followerPose.setLowSpkPose(77.0, 42.5, Math.toRadians(-12.0));
                followerPose.setLowSpkEnd(107.0, 39.5, Math.toRadians(-17.0));
            }
        }

        if (useFarStopPose) // use far stop pose
            followerPose.useFarStopPose();
        else // use close stop pose
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
        if (useFarStartPose) // use far start pose
            launcher.setLauncherReadyVelocity(FAR_LAUNCH_TARGET_VELOCITY);
        else
            launcher.setLauncherReadyVelocity(CLOSE_LAUNCH_TARGET_VELOCITY);
        launcher.setLauncherCloseVelocity(CLOSE_LAUNCH_TARGET_VELOCITY, CLOSE_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherFarVelocity(FAR_LAUNCH_TARGET_VELOCITY, FAR_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherCoolOffSec(LAUNCH_COOL_OFF_SECONDS);
        launcher.setLauncherOnSecAtIdle(LAUNCH_ON_SECOND_AT_IDLE);
        launcher.setFeederRunSec(FEEDER_RUN_SECONDS);
        launcher.setPanicRunSec(FEEDER_PANIC_SECONDS);

        // Initialize Digital sensor
        leftBallSensor.build(hardwareMap);
        rightBallSensor.build(hardwareMap);

        // Initialize Indicator light
        indicatorLight.build(hardwareMap);
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
        sensorTimer.resetTimer();
        setPathState(PathState.START_POSE);
        getNextAction();
    }

    @Override
    public void loop() {
        if (runTime.getElapsedTimeSeconds() > 30) { // stop after 30 seconds
            intakeMotor.setIntakeOff();
            launcher.setLauncherOff();
            indicatorLight.setIndicatorColor(IndicatorLight.IndicatorColor.OFF);
            terminateOpModeNow();
        } else {
            // Follower drives through the updated path chains
            updatePathState();
            // Reverse feeders periodically if intake is on
            feederPanic(FEEDER_PANIC_INTERVAL);
        }

        // Update follower pathing on every iteration
        follower.update();
        currentPose = follower.getPose(); // the follower current pose

        // Log to telemetry for debugging
        telemetry.addData("Run Time", "(%.4fs)", runTime.getElapsedTimeSeconds());
        telemetry.addData("Current Pose", "x(%.4f), y(%.4f), h(%.4f)",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Current Action", isNextAction(null) ? "None": nextAction.toString());
        telemetry.addLine("Follower action sequence remaining:");
        telemetry.addLine(followerSelectedAction.getActionLines());
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Path Completion", "(%.2f)", follower.getPathCompletion());
        telemetry.addData("Launch Count", launchCount);
        telemetry.addData("Panic Count", panicCount);
        telemetry.update();
    }

    void setPathState(PathState newPathState) {
        this.pathState = newPathState;
        if (newPathState == PathState.SCORE_POSE)
            launcherReady();
    }

    void updatePathState() {
        if (isNextAction(null)) {
            intakeMotor.setIntakeOff();
            launcher.setLauncherOff();
            follower.holdPoint(currentPose);
        } else {
            checkBallDetected(); // Check if ball is present for launching

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
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2Score, PATH_SPEED_SLOW);
                        setPathState(PathState.SCORE_POSE);
                    } else if (isNextAction(FollowerAction.STOP)) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2Stop);
                        setPathState(PathState.STOP_POSE);
                    } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_startPos2Gate);
                        setPathState(PathState.GATE_OPEN_POSE);
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
                            launchCloseShot(3, CLOSE_LAUNCH_INTERVAL_SECONDS);
                        else if (isNextAction(FollowerAction.FAR_LAUNCH))
                            launchFarShot(3, FAR_LAUNCH_INTERVAL_SECONDS);

                        intakeMotor.setIntakeOff();
                        getNextAction(); // get the next follower action

                        if (isNextAction(FollowerAction.STOP)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_scorePos2Stop);
                            setPathState(PathState.STOP_POSE);
                        } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_scorePos2Gate);
                            setPathState(PathState.GATE_OPEN_POSE);
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
                            if (isMidSpikeGrabbed) {
                                followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkEnd2Score);
                                setPathState(PathState.SCORE_POSE);
                            } else {
                                followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkEnd2Return);
                                setPathState(PathState.HIGH_SPIKE_RETURN_POSE);
                            }
                        } else if (isNextAction(FollowerAction.STOP)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkEnd2Stop);
                            setPathState(PathState.STOP_POSE);
                        } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkEnd2Gate);
                            setPathState(PathState.GATE_OPEN_POSE);
                        }
                    }
                    break;

                case HIGH_SPIKE_RETURN_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_highSpkReturn2Score);
                        setPathState(PathState.SCORE_POSE);
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
                            if (!useRedPose) // blue
                                followerPose.setScorePose(56.0, 15.0, Math.toRadians(127.0));
                            else // red
                                followerPose.setScorePose(88.0, 15.0, Math.toRadians(64.0)); // 65.0

                            if (isLowSpikeGrabbed) {
                                followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkEnd2Score);
                                setPathState(PathState.SCORE_POSE);
                            } else {
                                followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkEnd2Return);
                                setPathState(PathState.MID_SPIKE_RETURN_POSE);
                            }
                        } else if (isNextAction(FollowerAction.STOP)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkEnd2Stop);
                            setPathState(PathState.STOP_POSE);
                        } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_midSpkEnd2Gate);
                            setPathState(PathState.GATE_OPEN_POSE);
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
                        isLowSpikeGrabbed = true;
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
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkEnd2Score, PATH_SPEED_SLOW);
                            setPathState(PathState.SCORE_POSE);
                        } else if (isNextAction(FollowerAction.STOP)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkEnd2Stop);
                            setPathState(PathState.STOP_POSE);
                        } else if (isNextAction(FollowerAction.OPEN_GATE)) {
                            followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkEnd2Gate);
                            setPathState(PathState.GATE_OPEN_POSE);
                        }
                    }
                    break;

                case LOW_SPIKE_RETURN_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_lowSpkReturn2Score);
                        setPathState(PathState.SCORE_POSE);
                    }
                    break;

                case GATE_OPEN_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gatePos2Contact, GATE_SPEED);
                        setPathState(PathState.GATE_CONTACT_POSE);
                    }
                    break;

                case GATE_CONTACT_POSE:
                    if (!follower.isBusy()) {
                        followingPath(pathBuilder, FollowerPathBuilder::buildPaths_gateContact2Return);
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
                        launcher.setLauncherOff();
                        follower.holdPoint(currentPose);
                    }
                    break;
            }
        }
    }

    void getNextAction() { // get next action from the follower action sequence
        nextAction = followerSelectedAction.getNextAction();
    }

    boolean isNextAction(FollowerAction action) {
        return nextAction == action;
    }

    void followingPath(FollowerPathBuilder pb,
                       Function<FollowerPathBuilder, PathChain> buildPath, double pathSpeed) {
        follower.followPath(buildPath.apply(pb), pathSpeed, true);
    }

    void followingPath(FollowerPathBuilder pb, Function<FollowerPathBuilder, PathChain> buildPath) {
        followingPath(pb, buildPath, PATH_SPEED_NORMAL);
    }

    void checkBallDetected() {
        isBallDetected = leftBallSensor.isDetected() || rightBallSensor.isDetected();

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
    }

    void feederPanic(double interval) {
        if (isBallDetected) {
            if (intakeMotor.isBusy() && (sensorTimer.getElapsedTimeSeconds() > interval)) {
                if (!launcher.isBusy()) { // only set launcher panic if it is not busy
                    launcher.launch(false, false, false, true);
                }
                sensorTimer.resetTimer();
            }
        }
    }

    void launcherReady() {
        if (isBallDetected) {
            launcher.launcherOnAtIdle();
            launcher.launch(false, false, true, false);
        }
    }

    void launcherPanic() {
        intakeMotor.setIntakePanic() ;
        launcherWaitSeconds(INTAKE_PANIC_TIME);
        intakeMotor.setIntakeOn();
        launcherWaitSeconds(INTAKE_PANIC_WAIT);
    }

    void launcherWaitSeconds(double interval) {
        launchTimer.resetTimer();
        do { checkBallDetected(); } while (launchTimer.getElapsedTimeSeconds() < interval);
        checkBallDetected(); // Check if ball is present for launching
    }

    void launchCloseShot(int count, double interval) {
        panicCount = 0;
        launchCount = 0;

        launcher.launcherOnAtIdle(); // keep launcher on between consecutive launches
        checkBallDetected();
        for (int i = 0; i < count; i++) {
            if (!isBallDetected) {
                panicCount++;
                launcherPanic();
            }
            if (isBallDetected) {
                launchCount++;
                launcher.launchCloseShot();
                if (launchCount < count) {
                    launcherWaitSeconds(interval); // interval between launches
                }
            } else if ((launchCount == 0) && (panicCount > 1)) {
                launcher.launcherOffAtIdle();
                return; // if no ball detected after consecutive launcherPanic, set launcher off
            }
        }

        if (launchCount < count) {
            if (!isBallDetected) {
                panicCount++;
                launcherPanic();
            }
            if (isBallDetected) {
                launchCount++;
                launcher.launchCloseShot(); // last launch if still have ball
            }
        }
        launcher.launcherOffAtIdle(); // set launcher off after next launch
    }

    void launchFarShot(int count, double interval) {
        panicCount = 0;
        launchCount = 0;

        launcher.launcherOnAtIdle(); // keep launcher on between consecutive launches
        checkBallDetected();
        for (int i = 0; i < count; i++) {
            if (!isBallDetected) {
                panicCount++;
                launcherPanic();
            }
            if (isBallDetected) {
                launchCount++;
                launcher.launchFarShot();
                if (launchCount < count) {
                    launcherWaitSeconds(interval); // interval between launches
                }
            } else if ((launchCount == 0) && (panicCount > 1)) {
                launcher.launcherOffAtIdle();
                return; // if no ball detected after consecutive launcherPanic, set launcher off
            }
        }

        if (launchCount < count) {
            if (!isBallDetected) {
                panicCount++;
                launcherPanic();
            }
            if (isBallDetected) {
                launchCount++;
                launcher.launchFarShot(); // last launch if still have ball
            }
        }
        launcher.launcherOffAtIdle(); // set launcher off after next launch
    }
}