package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "bluePedroPathing", group = "Error404")
public class bluePedroPathing extends LinearOpMode {

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

    private final double DRIVE_GRAB_SPEED = 0.5;
    private final double INTAKE_POWER = 0.75;
    private final double INTAKE_PANIC_TIME = 0.15;

    private final double ClOSE_LAUNCH_TARGET_VELOCITY = 1300;
    private final double CLOSE_LAUNCH_MIN_VELOCITY    = 1275;
    private final double FAR_LAUNCH_TARGET_VELOCITY   = 1600;
    private final double FAR_LAUNCH_MIN_VELOCITY      = 1575;
    private final double FEEDER_RUN_SECONDS = 0.10;
    private final double LAUNCH_COOLOFF_SECONDS = 0.20;
    private final double LAUNCH_INTERVAL_SECONDS = 0.20;

    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private Pose currentPose;

    private enum PathState {
        START_POS,
        SCORE_POS,
        SHOOT_ARTIFACT,
        HIGH_SPIKE_POS,
        GRAB_HIGH_SPIKE,
        MID_SPIKE_POS,
        GRAB_MID_SPIKE,
        LOW_SPIKE_POS,
        GRAB_LOW_SPIKE,
        STOP_POS
    }
    PathState pathState;

    private final double X_GOAL_WALL = 25.0;
    private final double Y_GOAL_WALL = 128.0;
    private final double H_GOAL_WALL = 144.0;

    private final double X_SCORE_POINT = 55.0;
    private final double Y_SCORE_POINT = 87.5;
    private final double H_SCORE_POINT = 132.0;

    private final double X_HIGH_SPIKE_LINE_START = 44.0;
    private final double Y_HIGH_SPIKE_LINE_START = 83.5;
    private final double H_HIGH_SPIKE_LINE_START = 180.0;
    private final double X_HIGH_SPIKE_LINE_END = 15.0;
    private final double Y_HIGH_SPIKE_LINE_END = 83.5;
    private final double H_HIGH_SPIKE_LINE_END = 180.0;

    private final double X_MID_SPIKE_LINE_START = 44.0;
    private final double Y_MID_SPIKE_LINE_START = 59.5;
    private final double H_MID_SPIKE_LINE_START = 180.0;
    private final double X_MID_SPIKE_LINE_END = 9.0;
    private final double Y_MID_SPIKE_LINE_END = 59.5;
    private final double H_MID_SPIKE_LINE_END = 180.0;

    private final double X_LOW_SPIKE_LINE_START = 44.0;
    private final double Y_LOW_SPIKE_LINE_START = 35.5;
    private final double H_LOW_SPIKE_LINE_START = 180.0;
    private final double X_LOW_SPIKE_LINE_END = 9.0;
    private final double Y_LOW_SPIKE_LINE_END = 35.5;
    private final double H_LOW_SPIKE_LINE_END = 180.0;

    private final double X_STOP_POINT = 55.0;
    private final double Y_STOP_POINT = 48.0;
    private final double H_STOP_POINT = 0.0;

    // Initialize elapsed timer in milliseconds
    private final Timer runTime = new Timer();
    private final Timer pathTimer = new Timer();
    private final Timer panicTimer = new Timer();

    // Initialize spike count
    private int spkCount = 0;
    private int spkLimit = 3;

    // Start Pose. Touching the Goal wall with the robot's left rear outer wheel on the tile intersection.
    private final Pose startPose = new Pose(X_GOAL_WALL, Y_GOAL_WALL, Math.toRadians(H_GOAL_WALL));

    // Scoring Pose. Facing the Goal at a 132 degree angle.
    private final Pose scorePose = new Pose(X_SCORE_POINT, Y_SCORE_POINT, Math.toRadians(H_SCORE_POINT));

    // Grab Pose of the Highest (1st Set) of Artifacts from the Spike Mark.
    private final Pose highSpkPose = new Pose(X_HIGH_SPIKE_LINE_START, Y_HIGH_SPIKE_LINE_START, Math.toRadians(H_HIGH_SPIKE_LINE_START));
    private final Pose highSpkEnd = new Pose(X_HIGH_SPIKE_LINE_END, Y_HIGH_SPIKE_LINE_END, Math.toRadians(H_HIGH_SPIKE_LINE_END));

    // Grab Pose of the Middle (2nd Set) of Artifacts from the Spike Mark.
    private final Pose midSpkPose = new Pose(X_MID_SPIKE_LINE_START, Y_MID_SPIKE_LINE_START, Math.toRadians(H_MID_SPIKE_LINE_START));
    private final Pose midSpkEnd = new Pose(X_MID_SPIKE_LINE_END, Y_MID_SPIKE_LINE_END, Math.toRadians(H_MID_SPIKE_LINE_END));

    // Grab Pose of the Lowest (3rd Set)of Artifacts from the Spike Mark.
    private final Pose lowSpkPose = new Pose(X_LOW_SPIKE_LINE_START, Y_LOW_SPIKE_LINE_START, Math.toRadians(H_LOW_SPIKE_LINE_START));
    private final Pose lowSpkEnd = new Pose(X_LOW_SPIKE_LINE_END, Y_LOW_SPIKE_LINE_END, Math.toRadians(H_LOW_SPIKE_LINE_END));

    private final Pose stopPose = new Pose(X_STOP_POINT, Y_STOP_POINT, Math.toRadians(H_STOP_POINT));

    // Initialize various Paths
    private PathChain startPos2ScorePos;

    private PathChain scorePos2HighSpkPos;
    private PathChain highSpkPos2GrabEnd;
    private PathChain highSpkGrbEnd2ScorePos;

    private PathChain scorePos2MidSpkPos;
    private PathChain midSpkPos2GrabEnd;
    private PathChain midSpkGrbEnd2ScorePos;

    private PathChain scorePos2LowSpkPos;
    private PathChain lowSpkPos2GrabEnd;
    private PathChain lowSpkGrbEnd2ScorePos;

    private PathChain scorePos2StopPos;

    public void buildPaths_startPos2Score() {
        startPos2ScorePos = follower.pathBuilder() // move from start position to scoring position
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPaths_grabHighSpkSet2Score() {
        scorePos2HighSpkPos = follower.pathBuilder() // move from scoring position to highest spike mark position
                .addPath(new BezierLine(scorePose, highSpkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), highSpkPose.getHeading())
                .build();

        highSpkPos2GrabEnd = follower.pathBuilder() // grab the highest spike mark artifacts
                .addPath(new BezierLine(highSpkPose, highSpkEnd))
                .setLinearHeadingInterpolation(highSpkPose.getHeading(), highSpkEnd.getHeading())
                .build();

        highSpkGrbEnd2ScorePos = follower.pathBuilder() // move back to scoring position
                .addPath(new BezierLine(highSpkEnd, scorePose))
                .setLinearHeadingInterpolation(highSpkEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPaths_grabMidSpkSet2Score() {
        scorePos2MidSpkPos = follower.pathBuilder() // move from scoring position to middle spike mark position
                .addPath(new BezierLine(scorePose, midSpkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), midSpkPose.getHeading())
                .build();

        midSpkPos2GrabEnd = follower.pathBuilder() // grab the middle spike mark artifacts
                .addPath(new BezierLine(midSpkPose, midSpkEnd))
                .setLinearHeadingInterpolation(midSpkPose.getHeading(), midSpkEnd.getHeading())
                .build();

        midSpkGrbEnd2ScorePos = follower.pathBuilder() // move back to scoring position
                .addPath(new BezierLine(midSpkEnd, scorePose))
                .setLinearHeadingInterpolation(midSpkEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPaths_grabLowSpkSet2Score() {
        scorePos2LowSpkPos = follower.pathBuilder() // move from scoring position to lowest spike mark position
                .addPath(new BezierLine(scorePose, lowSpkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), midSpkPose.getHeading())
                .build();

        lowSpkPos2GrabEnd = follower.pathBuilder() // grab the lowest spike mark artifacts
                .addPath(new BezierLine(lowSpkPose, lowSpkEnd))
                .setLinearHeadingInterpolation(lowSpkPose.getHeading(), lowSpkEnd.getHeading())
                .build();

        lowSpkGrbEnd2ScorePos = follower.pathBuilder() // move back to scoring position
                .addPath(new BezierLine(lowSpkEnd, scorePose))
                .setLinearHeadingInterpolation(lowSpkEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPaths_scorePos2StopPos() {
        scorePos2StopPos = follower.pathBuilder()// move from scoring position to stop position
                .addPath(new BezierLine(scorePose, stopPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), stopPose.getHeading())
                .build();
    }

    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize intake motor
        intakeMotor.build(hardwareMap);
        intakeMotor.setPower(INTAKE_POWER);
        intakeMotor.setPanicTime(INTAKE_PANIC_TIME);

        // Initialize launcher and feeders
        launcher.build(hardwareMap);
        launcher.setLauncherCloseVelocity(ClOSE_LAUNCH_TARGET_VELOCITY, CLOSE_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherFarVelocity(FAR_LAUNCH_TARGET_VELOCITY, FAR_LAUNCH_MIN_VELOCITY);
        launcher.setLauncherCoolOffSec(LAUNCH_COOLOFF_SECONDS);
        launcher.setFeederRunSec(FEEDER_RUN_SECONDS);

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build paths
        buildPaths_startPos2Score();
        buildPaths_grabHighSpkSet2Score();
        buildPaths_grabMidSpkSet2Score();
        buildPaths_grabLowSpkSet2Score();
        buildPaths_scorePos2StopPos();

        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        waitForStart();
        setPathState(PathState.START_POS);

        setSpkLimit(spkLimit);
        runTime.resetTimer();

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose

            // Follower drive through the given pathing
            updatePathState();
        }

        // Log to Panels and driver station (custom log function)
        log("RunTime", runTime.toString());
        log("X-Position", currentPose.getX());
        log("Y-Position", currentPose.getY());
        log("Heading", currentPose.getHeading());
        log("PathState", pathState.toString());
        log("PathTimer", pathTimer.getElapsedTimeSeconds());
        telemetry.update(); // Update the driver station after logging
    }

    public void resetSpkCount() {
        spkCount = 0;
    }

    public void setSpkLimit(int limit) {
        spkLimit = limit;
        resetSpkCount();
    }

    public int getSpkLimit() {
        return spkLimit;
    }

    public boolean isSpkLimitHit() {
        return ++spkCount > spkLimit;
    }

    public void setPathState(PathState newPathState) {
        this.pathState = newPathState;
        pathTimer.resetTimer();
    }

    public void updatePathState() {
        switch (pathState) {
            case START_POS:
                follower.followPath(startPos2ScorePos);
                setPathState(PathState.SCORE_POS);
                break;

            case SCORE_POS:
                if (!follower.isBusy()) { // wait till follower path update is done
                    intakeMotor.setIntakeOn();
                    setPathState(PathState.SHOOT_ARTIFACT);
                }
                break;

            case SHOOT_ARTIFACT:
                customLaunchCloseShot(3, LAUNCH_INTERVAL_SECONDS);  // intake panic + close shot
                intakeMotor.setIntakeOff();

                if (isSpkLimitHit()) {
                    follower.followPath(scorePos2StopPos);
                    setPathState(PathState.STOP_POS);
                } else if (spkCount == 1) {
                    follower.followPath(scorePos2HighSpkPos);
                    setPathState(PathState.HIGH_SPIKE_POS);
                } else if (spkCount == 2) {
                    follower.followPath(scorePos2MidSpkPos);
                    setPathState(PathState.MID_SPIKE_POS);
                } else if (spkCount == 3) {
                    follower.followPath(scorePos2LowSpkPos);
                    setPathState(PathState.LOW_SPIKE_POS);
                } else {
                    follower.followPath(scorePos2HighSpkPos);
                    setPathState(PathState.STOP_POS);
                }
                break;

            case HIGH_SPIKE_POS:
                if (!follower.isBusy()) {
                    intakeMotor.setIntakeOn();
                    follower.followPath(highSpkPos2GrabEnd, DRIVE_GRAB_SPEED, true);
                    setPathState(PathState.GRAB_HIGH_SPIKE);
                }
                break;

            case GRAB_HIGH_SPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(highSpkGrbEnd2ScorePos);
                    setPathState(PathState.SCORE_POS);
                }
                break;

            case MID_SPIKE_POS:
                if (!follower.isBusy()) {
                    intakeMotor.setIntakeOn();
                    follower.followPath(midSpkPos2GrabEnd, DRIVE_GRAB_SPEED, true);
                    setPathState(PathState.GRAB_MID_SPIKE);
                }
                break;

            case GRAB_MID_SPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(midSpkGrbEnd2ScorePos);
                    setPathState(PathState.SCORE_POS);
                }
                break;

            case LOW_SPIKE_POS:
                if (!follower.isBusy()) {
                    intakeMotor.setIntakeOn();
                    follower.followPath(lowSpkPos2GrabEnd, DRIVE_GRAB_SPEED, true);
                    setPathState(PathState.GRAB_LOW_SPIKE);
                }
                break;

            case GRAB_LOW_SPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(lowSpkGrbEnd2ScorePos);
                    setPathState(PathState.SCORE_POS);
                }
                break;

            case STOP_POS:
                if (!follower.isBusy()) {
                    intakeMotor.setIntakeOff();
                    follower.holdPoint(stopPose);
                }
                break;
        }
    }

    private void customLaunchCloseShot() {
        panicTimer.resetTimer();
        intakeMotor.setIntakePanic(INTAKE_PANIC_TIME);
        launcher.launchCloseShot();
    }

    private void customLaunchCloseShot(int count, double interval) {
        for (int i = 0; i < count; i++) {
            do {} while (pathTimer.getElapsedTimeSeconds() < interval);
            customLaunchCloseShot();
        }
    }

    private void customLaunchFarShot() {
        panicTimer.resetTimer();
        intakeMotor.setIntakePanic(INTAKE_PANIC_TIME);
        launcher.launchFarShot();
    }

    private void customLaunchFarShot(int count, double interval) {
        for (int i = 0; i < count; i++) {
            do {} while (pathTimer.getElapsedTimeSeconds() < interval);
            customLaunchFarShot();
        }
    }

    // Custom logging function to support telemetry and Panels
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }
}
