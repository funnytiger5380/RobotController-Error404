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
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.mechanisms.DigitalSensor;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.LinkedList;

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

    private final DigitalSensor ballSensor = new DigitalSensor()
            .sensorName("ball_sensor")
            .sensorMode(DigitalChannel.Mode.INPUT);

    private final double PATH_SPEED = 0.75;
    private final double GRAB_SPEED = 0.35;
    private final double GATE_SPEED = 0.35;
    private final double INTAKE_POWER = 0.75;
    private final double INTAKE_PANIC_TIME = 0.15;

    private final double ClOSE_LAUNCH_TARGET_VELOCITY = 1300;
    private final double CLOSE_LAUNCH_MIN_VELOCITY    = 1280;
    private final double FAR_LAUNCH_TARGET_VELOCITY   = 1600;
    private final double FAR_LAUNCH_MIN_VELOCITY      = 1580;
    private final double FEEDER_RUN_SECONDS = 0.10;
    private final double LAUNCH_COOL_OFF_SECONDS = 0.20;
    private final double LAUNCH_INTERVAL_SECONDS = 0.15;

    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private Pose currentPose;

    private enum PathState {
        START_POSE, SCORE_POSE, HIGH_SPIKE_POSE, GRAB_HIGH_SPIKE, MID_SPIKE_POSE, GRAB_MID_SPIKE,
        LOW_SPIKE_POSE, GRAB_LOW_SPIKE, GATE_OPEN_UP_POSE, GATE_CONTACT_UP_POSE,
        GATE_OPEN_DOWN_POSE, GATE_CONTACT_DOWN_POSE, GATE_RETURN_POSE, STOP_POSE
    } PathState pathState;

    private enum FollowerAction {
        GO_SHOOT, GRAB_HIGH_SPIKE, GRAB_MID_SPIKE, GRAB_LOW_SPIKE, OPEN_GATE, STOP
    } FollowerAction followerAction;

    LinkedList<bluePedroPathing.FollowerAction> followerActionList;

    private final double X_GOAL_WALL = 24.5;
    private final double Y_GOAL_WALL = 126.5;
    private final double H_GOAL_WALL = 144.0;

    private final double X_SCORE_POSE = 55.0;
    private final double Y_SCORE_POSE = 91.0;
    private final double H_SCORE_POSE = 140.0;

    private final double X_GATE_POSE = 17.5;
    private final double Y_GATE_POSE = 69.0;
    private final double H_GATE_POSE_UP = 90.0;
    private final double H_GATE_POSE_DW = -90.0;
    private final double X_GATE_CONTACT = 14;
    private final double Y_GATE_CONTACT = 69.0;
    private final double H_GATE_CONTACT_UP = 90.0;
    private final double H_GATE_CONTACT_DW = -90.0;
    private final double X_GATE_RETURN = 44.0;
    private final double Y_GATE_RETURN = 69.0;
    private final double H_GATE_RETURN = 20.0;

    private final double X_HIGH_SPIKE_LINE_START = 44.0;
    private final double Y_HIGH_SPIKE_LINE_START = 84.0;
    private final double H_HIGH_SPIKE_LINE_START = 180.0;
    private final double X_HIGH_SPIKE_LINE_END = 17.5;
    private final double Y_HIGH_SPIKE_LINE_END = 84.0;
    private final double H_HIGH_SPIKE_LINE_END = 180.0;

    private final double X_MID_SPIKE_LINE_START = 44.0;
    private final double Y_MID_SPIKE_LINE_START = 60.0;
    private final double H_MID_SPIKE_LINE_START = 180.0;
    private final double X_MID_SPIKE_LINE_END = 16.5;
    private final double Y_MID_SPIKE_LINE_END = 60.0;
    private final double H_MID_SPIKE_LINE_END = 180.0;

    private final double X_LOW_SPIKE_LINE_START = 44.0;
    private final double Y_LOW_SPIKE_LINE_START = 36.0;
    private final double H_LOW_SPIKE_LINE_START = 180.0;
    private final double X_LOW_SPIKE_LINE_END = 16.5;
    private final double Y_LOW_SPIKE_LINE_END = 36.0;
    private final double H_LOW_SPIKE_LINE_END = 180.0;

    private final double X_STOP_POINT = 55.0;
    private final double Y_STOP_POINT = 69.0;
    private final double H_STOP_POINT = 90.0;

    // Initialize elapsed timer in milliseconds
    private final Timer runTime = new Timer();
    private final Timer pathTimer = new Timer();
    private final Timer auxTimer = new Timer();

    // Start Pose. Touching the Goal wall with the robot's left rear outer wheel on the tile intersection.
    private final Pose startPose = new Pose(X_GOAL_WALL, Y_GOAL_WALL, Math.toRadians(H_GOAL_WALL));

    // Stop Pose at which the robot finishes its autonomous path.
    private final Pose stopPose = new Pose(X_STOP_POINT, Y_STOP_POINT, Math.toRadians(H_STOP_POINT));

    // Scoring Pose. Facing the Goal at a 132 degree angle.
    private final Pose scorePose = new Pose(X_SCORE_POSE, Y_SCORE_POSE, Math.toRadians(H_SCORE_POSE));

    // Grab Pose of the Highest (1st Set) of Artifacts from the Spike Mark.
    private final Pose highSpkPose = new Pose(X_HIGH_SPIKE_LINE_START, Y_HIGH_SPIKE_LINE_START, Math.toRadians(H_HIGH_SPIKE_LINE_START));
    private final Pose highSpkEnd = new Pose(X_HIGH_SPIKE_LINE_END, Y_HIGH_SPIKE_LINE_END, Math.toRadians(H_HIGH_SPIKE_LINE_END));

    // Grab Pose of the Middle (2nd Set) of Artifacts from the Spike Mark.
    private final Pose midSpkPose = new Pose(X_MID_SPIKE_LINE_START, Y_MID_SPIKE_LINE_START, Math.toRadians(H_MID_SPIKE_LINE_START));
    private final Pose midSpkEnd = new Pose(X_MID_SPIKE_LINE_END, Y_MID_SPIKE_LINE_END, Math.toRadians(H_MID_SPIKE_LINE_END));

    // Grab Pose of the Lowest (3rd Set)of Artifacts from the Spike Mark.
    private final Pose lowSpkPose = new Pose(X_LOW_SPIKE_LINE_START, Y_LOW_SPIKE_LINE_START, Math.toRadians(H_LOW_SPIKE_LINE_START));
    private final Pose lowSpkEnd = new Pose(X_LOW_SPIKE_LINE_END, Y_LOW_SPIKE_LINE_END, Math.toRadians(H_LOW_SPIKE_LINE_END));

    // Gate start and return poses to flex the gate open.
    private final Pose gatePoseUp = new Pose(X_GATE_POSE, Y_GATE_POSE, Math.toRadians(H_GATE_POSE_UP));
    private final Pose gatePoseDw = new Pose(X_GATE_POSE, Y_GATE_POSE, Math.toRadians(H_GATE_POSE_DW));
    private final Pose gateContactUp = new Pose(X_GATE_CONTACT, Y_GATE_CONTACT, Math.toRadians(H_GATE_CONTACT_UP));
    private final Pose gateContactDw = new Pose(X_GATE_CONTACT, Y_GATE_CONTACT, Math.toRadians(H_GATE_CONTACT_DW));
    private final Pose gateReturn = new Pose(X_GATE_RETURN, Y_GATE_RETURN, Math.toRadians(H_GATE_RETURN));

    // Build various path chains
    public PathChain buildPaths_startPos2Score() {
        return follower.pathBuilder() // move from start position to scoring position
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2Stop() {
        return follower.pathBuilder() // move from start position to stop position
                .addPath(new BezierLine(startPose, stopPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2Gate() {
        return follower.pathBuilder() // move from start position to gate open standby position
                .addPath(new BezierLine(startPose, gatePoseUp))
                .setLinearHeadingInterpolation(startPose.getHeading(), gatePoseUp.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2HighSpkSet() {
        return follower.pathBuilder() // move from start position to high spike mark position
                .addPath(new BezierLine(startPose, highSpkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), highSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2MidSpkSet() {
        return follower.pathBuilder() // move from start position to middle spike mark position
                .addPath(new BezierLine(startPose, midSpkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), midSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2LowSpkSet() {
        return follower.pathBuilder() // move from start position to low spike mark position
                .addPath(new BezierLine(startPose, lowSpkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), lowSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2Stop() {
        return follower.pathBuilder()// move from scoring position to stop position
                .addPath(new BezierLine(scorePose, stopPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2Gate() {
        return follower.pathBuilder()// move from scoring position to gate open standby position
                .addPath(new BezierLine(scorePose, gatePoseUp))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePoseUp.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2HighSpk() {
        return follower.pathBuilder() // move from scoring position to highest spike mark position
                .addPath(new BezierLine(scorePose, highSpkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), highSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2MidSpk() {
        return follower.pathBuilder() // move from scoring position to middle spike mark position
                .addPath(new BezierLine(scorePose, midSpkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), midSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2LowSpk() {
        return follower.pathBuilder() // move from scoring position to lowest spike mark position
                .addPath(new BezierLine(scorePose, lowSpkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lowSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_highSpkPos2GrabEnd() {
        return follower.pathBuilder() // grab the highest spike mark artifacts
                .addPath(new BezierLine(highSpkPose, highSpkEnd))
                .setLinearHeadingInterpolation(highSpkPose.getHeading(), highSpkEnd.getHeading())
                .build();
    }

    public PathChain buildPaths_midSpkPos2GrabEnd() {
        return follower.pathBuilder() // grab the middle spike mark artifacts
                .addPath(new BezierLine(midSpkPose, midSpkEnd))
                .setLinearHeadingInterpolation(midSpkPose.getHeading(), midSpkEnd.getHeading())
                .build();
    }

    public PathChain buildPaths_lowSpkPos2GrabEnd() {
        return follower.pathBuilder() // grab the lowest spike mark artifacts
                .addPath(new BezierLine(lowSpkPose, lowSpkEnd))
                .setLinearHeadingInterpolation(lowSpkPose.getHeading(), lowSpkEnd.getHeading())
                .build();
    }

    public PathChain buildPaths_highSpkEnd2Score() {
        return follower.pathBuilder() // move back to scoring position
                .addPath(new BezierLine(highSpkEnd, scorePose))
                .setLinearHeadingInterpolation(highSpkEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_midSpkEnd2Score() {
        return follower.pathBuilder() // move back to scoring position
                .addPath(new BezierLine(midSpkEnd, scorePose))
                .setLinearHeadingInterpolation(midSpkEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_lowSpkEnd2Score() {
        return follower.pathBuilder() // move back to scoring position
                .addPath(new BezierLine(lowSpkEnd, scorePose))
                .setLinearHeadingInterpolation(lowSpkEnd.getHeading(), scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_highSpkEnd2Gate() {
        return follower.pathBuilder() // move to gate open standby position
                .addPath(new BezierLine(highSpkEnd, gatePoseUp))
                .setLinearHeadingInterpolation(highSpkEnd.getHeading(), gatePoseUp.getHeading())
                .build();
    }

    public PathChain buildPaths_midSpkEnd2Gate() {
        return follower.pathBuilder() // move to gate open standby position
                .addPath(new BezierLine(midSpkEnd, gatePoseDw))
                .setLinearHeadingInterpolation(midSpkEnd.getHeading(), gatePoseDw.getHeading())
                .build();
    }

    public PathChain buildPaths_lowSpkEnd2Gate() {
        return follower.pathBuilder() // move to gate open standby position
                .addPath(new BezierLine(lowSpkEnd, gatePoseDw))
                .setLinearHeadingInterpolation(lowSpkEnd.getHeading(), gatePoseDw.getHeading())
                .build();
    }

    public PathChain buildPaths_highSpkEnd2Stop() {
        return follower.pathBuilder() // move to stop position
                .addPath(new BezierLine(highSpkEnd, stopPose))
                .setLinearHeadingInterpolation(highSpkEnd.getHeading(), stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_midSpkEnd2Stop() {
        return follower.pathBuilder() // move to stop position
                .addPath(new BezierLine(midSpkEnd, stopPose))
                .setLinearHeadingInterpolation(midSpkEnd.getHeading(), stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_lowSpkEnd2Stop() {
        return follower.pathBuilder() // move to stop position
                .addPath(new BezierLine(lowSpkEnd, stopPose))
                .setLinearHeadingInterpolation(lowSpkEnd.getHeading(), stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_gatePosUp2Contact() {
        return follower.pathBuilder() // move from gate open standby position to contact position
                .addPath(new BezierLine(gatePoseUp, gateContactUp))
                .setLinearHeadingInterpolation(gatePoseUp.getHeading(), gateContactUp.getHeading())
                .build();
    }

    public PathChain buildPaths_gatePosDw2Contact() {
        return follower.pathBuilder() // move from gate open standby position to contact position
                .addPath(new BezierLine(gatePoseDw, gateContactDw))
                .setLinearHeadingInterpolation(gatePoseDw.getHeading(), gateContactDw.getHeading())
                .build();
    }

    public PathChain buildPaths_gateContactUp2Return() {
        return follower.pathBuilder() // move from contact position to gate return position
                .addPath(new BezierLine(gateContactUp, gateReturn))
                .setLinearHeadingInterpolation(gateContactUp.getHeading(), gateReturn.getHeading())
                .build();
    }

    public PathChain buildPaths_gateContactDw2Return() {
        return follower.pathBuilder() // move from contact position to gate return position
                .addPath(new BezierLine(gateContactDw, gateReturn))
                .setLinearHeadingInterpolation(gateContactDw.getHeading(), gateReturn.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturn2Score() {
        return follower.pathBuilder() // move from gate return position to scoring position
                .addPath(new BezierLine(gateReturn, scorePose))
                .setLinearHeadingInterpolation(gateReturn.getHeading(), scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturnPos2Stop() {
        return follower.pathBuilder() // move from gate return position to stop position
                .addPath(new BezierLine(gateReturn, stopPose))
                .setLinearHeadingInterpolation(gateReturn.getHeading(), stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturnPos2HighSpk() {
        return follower.pathBuilder() // move from gate return position to highest spike mark position
                .addPath(new BezierLine(gateReturn, highSpkPose))
                .setLinearHeadingInterpolation(gateReturn.getHeading(), highSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturnPos2MidSpk() {
        return follower.pathBuilder() // move from gate return position to middle spike mark position
                .addPath(new BezierLine(gateReturn, midSpkPose))
                .setLinearHeadingInterpolation(gateReturn.getHeading(), midSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturnPos2LowSpk() {
        return follower.pathBuilder() // move from gate return position to lowest spike mark position
                .addPath(new BezierLine(gateReturn, lowSpkPose))
                .setLinearHeadingInterpolation(gateReturn.getHeading(), lowSpkPose.getHeading())
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
        launcher.setLauncherCoolOffSec(LAUNCH_COOL_OFF_SECONDS);
        launcher.setFeederRunSec(FEEDER_RUN_SECONDS);

        // Initialize Digital sensor
        ballSensor.build(hardwareMap);

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize follower action list
        followerActionList = new LinkedList<>();
        followerActionList.add(FollowerAction.GO_SHOOT);
        followerActionList.add(FollowerAction.GRAB_MID_SPIKE);
        followerActionList.add(FollowerAction.OPEN_GATE);
        followerActionList.add(FollowerAction.GO_SHOOT);
        followerActionList.add(FollowerAction.GRAB_HIGH_SPIKE);
        followerActionList.add(FollowerAction.GO_SHOOT);
        followerActionList.add(FollowerAction.GRAB_LOW_SPIKE);
        followerActionList.add(FollowerAction.GO_SHOOT);
        followerActionList.add(FollowerAction.STOP);

        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        waitForStart();
        runTime.resetTimer();
        nextFollowerAction();
        setPathState(PathState.START_POSE);

        while (opModeIsActive()) {
            //if (runTime.getElapsedTimeSeconds() > 30) // Stop after 30 seconds
            //    terminateOpModeNow();
            //else
                updatePathState(); // Follower drives through the given pathing

            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Get the current pose

            // Log to Panels and driver station (custom log function)
            log("RunTime", runTime.toString());
            log("X-Position", currentPose.getX());
            log("Y-Position", currentPose.getY());
            log("Heading", Math.toDegrees(currentPose.getHeading()));
            log("PathState", pathState.toString());
            log("PathTimer", pathTimer.getElapsedTimeSeconds());
            telemetry.update(); // Update the driver station after logging
        }
    }

    public void setPathState(PathState newPathState) {
        this.pathState = newPathState;
        pathTimer.resetTimer();
    }

    public void updatePathState() {
        if (followerAction == null) {
            intakeMotor.setIntakeOff();
            launcher.setLauncherOff();
            follower.holdPoint(currentPose);
        } else {
            switch (pathState) {
                case START_POSE:
                    if (followerAction == FollowerAction.GO_SHOOT) {
                        intakeMotor.setIntakeOn();
                        follower.followPath(buildPaths_startPos2Score(),PATH_SPEED,true);
                        setPathState(PathState.SCORE_POSE);
                    } else if (followerAction == FollowerAction.STOP) {
                        follower.followPath(buildPaths_startPos2Stop(),PATH_SPEED,true);
                        setPathState(PathState.STOP_POSE);
                    } else if (followerAction == FollowerAction.OPEN_GATE) {
                        follower.followPath(buildPaths_startPos2Gate(),PATH_SPEED,true);
                        setPathState(PathState.GATE_OPEN_UP_POSE);
                    } else if (followerAction == FollowerAction.GRAB_HIGH_SPIKE) {
                        follower.followPath(buildPaths_startPos2HighSpkSet(),PATH_SPEED,true);
                        setPathState(PathState.HIGH_SPIKE_POSE);
                    } else if (followerAction == FollowerAction.GRAB_MID_SPIKE) {
                        follower.followPath(buildPaths_startPos2MidSpkSet(),PATH_SPEED,true);
                        setPathState(PathState.MID_SPIKE_POSE);
                    } else if (followerAction == FollowerAction.GRAB_LOW_SPIKE) {
                        follower.followPath(buildPaths_startPos2LowSpkSet(),PATH_SPEED,true);
                        setPathState(PathState.LOW_SPIKE_POSE);
                    }
                    break;

                case SCORE_POSE:
                    if (!follower.isBusy()) { // wait till follower path update is done
                        customLaunchCloseShot(3, LAUNCH_INTERVAL_SECONDS);  // intake panic + close shot
                        intakeMotor.setIntakeOff();
                        nextFollowerAction(); // get the next follower action

                        if (followerAction == FollowerAction.STOP) {
                            follower.followPath(buildPaths_scorePos2Stop(),PATH_SPEED,true);
                            setPathState(PathState.STOP_POSE);
                        } else if (followerAction == FollowerAction.OPEN_GATE) {
                            follower.followPath(buildPaths_scorePos2Gate(),PATH_SPEED,true);
                            setPathState(PathState.GATE_OPEN_UP_POSE);
                        } else if (followerAction == FollowerAction.GRAB_HIGH_SPIKE) {
                            follower.followPath(buildPaths_scorePos2HighSpk(),PATH_SPEED,true);
                            setPathState(PathState.HIGH_SPIKE_POSE);
                        } else if (followerAction == FollowerAction.GRAB_MID_SPIKE) {
                            follower.followPath(buildPaths_scorePos2MidSpk(),PATH_SPEED,true);
                            setPathState(PathState.MID_SPIKE_POSE);
                        } else if (followerAction == FollowerAction.GRAB_LOW_SPIKE) {
                            follower.followPath(buildPaths_scorePos2LowSpk(),PATH_SPEED,true);
                            setPathState(PathState.LOW_SPIKE_POSE);
                        }
                    }
                    break;

                case HIGH_SPIKE_POSE:
                    if (!follower.isBusy()) {
                        intakeMotor.setIntakeOn();
                        follower.followPath(buildPaths_highSpkPos2GrabEnd(),GRAB_SPEED,true);
                        setPathState(PathState.GRAB_HIGH_SPIKE);
                    }
                    break;

                case MID_SPIKE_POSE:
                    if (!follower.isBusy()) {
                        intakeMotor.setIntakeOn();
                        follower.followPath(buildPaths_midSpkPos2GrabEnd(),GRAB_SPEED,true);
                        setPathState(PathState.GRAB_MID_SPIKE);
                    }
                    break;

                case LOW_SPIKE_POSE:
                    if (!follower.isBusy()) {
                        intakeMotor.setIntakeOn();
                        follower.followPath(buildPaths_lowSpkPos2GrabEnd(),GRAB_SPEED,true);
                        setPathState(PathState.GRAB_LOW_SPIKE);
                    }
                    break;

                case GRAB_HIGH_SPIKE:
                    if (!follower.isBusy()) {
                        nextFollowerAction(); // get the next follower action

                        if (followerAction == FollowerAction.GO_SHOOT) {
                            follower.followPath(buildPaths_highSpkEnd2Score(),PATH_SPEED,true);
                            setPathState(PathState.SCORE_POSE);
                        } else if (followerAction == FollowerAction.STOP) {
                            follower.followPath(buildPaths_highSpkEnd2Stop(),PATH_SPEED,true);
                            setPathState(PathState.STOP_POSE);
                        } else if (followerAction == FollowerAction.OPEN_GATE) {
                            follower.followPath(buildPaths_highSpkEnd2Gate(),PATH_SPEED,true);
                            setPathState(PathState.GATE_OPEN_UP_POSE);
                        }
                    }
                    break;

                case GRAB_MID_SPIKE:
                    if (!follower.isBusy()) {
                        nextFollowerAction(); // get the next follower action

                        if (followerAction == FollowerAction.GO_SHOOT) {
                            follower.followPath(buildPaths_midSpkEnd2Score(),PATH_SPEED,true);
                            setPathState(PathState.SCORE_POSE);
                        } else if (followerAction == FollowerAction.STOP) {
                            follower.followPath(buildPaths_midSpkEnd2Stop(),PATH_SPEED,true);
                            setPathState(PathState.STOP_POSE);
                        } else if (followerAction == FollowerAction.OPEN_GATE) {
                            follower.followPath(buildPaths_midSpkEnd2Gate(),PATH_SPEED,true);
                            setPathState(PathState.GATE_OPEN_DOWN_POSE);
                        }
                    }
                    break;

                case GRAB_LOW_SPIKE:
                    if (!follower.isBusy()) {
                        nextFollowerAction(); // get the next follower action

                        if (followerAction == FollowerAction.GO_SHOOT) {
                            follower.followPath(buildPaths_lowSpkEnd2Score(),PATH_SPEED,true);
                        } else if (followerAction == FollowerAction.STOP) {
                            follower.followPath(buildPaths_lowSpkEnd2Stop(),PATH_SPEED,true);
                            setPathState(PathState.STOP_POSE);
                        } else if (followerAction == FollowerAction.OPEN_GATE) {
                            follower.followPath(buildPaths_lowSpkEnd2Gate(),PATH_SPEED,true);
                            setPathState(PathState.GATE_OPEN_DOWN_POSE);
                        }
                    }
                    break;

                case GATE_OPEN_UP_POSE:
                    if (!follower.isBusy()) {
                        follower.followPath(buildPaths_gatePosUp2Contact(),GATE_SPEED,true);
                        setPathState(PathState.GATE_CONTACT_UP_POSE);
                    }
                    break;

                case GATE_OPEN_DOWN_POSE:
                    if (!follower.isBusy()) {
                        follower.followPath(buildPaths_gatePosDw2Contact(),GATE_SPEED,true);
                        setPathState(PathState.GATE_CONTACT_DOWN_POSE);
                    }
                    break;

                case GATE_CONTACT_UP_POSE:
                    if (!follower.isBusy()) {
                        follower.followPath(buildPaths_gateContactUp2Return(),PATH_SPEED,true);
                        setPathState(PathState.GATE_RETURN_POSE);
                    }
                    break;

                case GATE_CONTACT_DOWN_POSE:
                    if (!follower.isBusy()) {
                        follower.followPath(buildPaths_gateContactDw2Return(),PATH_SPEED,true);
                        setPathState(PathState.GATE_RETURN_POSE);
                    }
                    break;

                case GATE_RETURN_POSE:
                    if (!follower.isBusy()) {
                        nextFollowerAction();

                        if (followerAction == FollowerAction.GO_SHOOT) {
                            follower.followPath(buildPaths_gateReturn2Score(),PATH_SPEED,true);
                            setPathState(PathState.SCORE_POSE);
                        } else if (followerAction == FollowerAction.STOP) {
                            follower.followPath(buildPaths_gateReturnPos2Stop(),PATH_SPEED,true);
                            setPathState(PathState.STOP_POSE);
                        } else if (followerAction == FollowerAction.GRAB_HIGH_SPIKE) {
                            follower.followPath(buildPaths_gateReturnPos2HighSpk(),PATH_SPEED,true);
                            setPathState(PathState.HIGH_SPIKE_POSE);
                        } else if (followerAction == FollowerAction.GRAB_MID_SPIKE) {
                            follower.followPath(buildPaths_gateReturnPos2MidSpk(),PATH_SPEED,true);
                            setPathState(PathState.MID_SPIKE_POSE);
                        } else if (followerAction == FollowerAction.GRAB_LOW_SPIKE) {
                            follower.followPath(buildPaths_gateReturnPos2LowSpk(),PATH_SPEED,true);
                            setPathState(PathState.LOW_SPIKE_POSE);
                        }
                    }
                    break;

                case STOP_POSE:
                    if (!follower.isBusy()) {
                        intakeMotor.setIntakeOff();
                        follower.holdPoint(stopPose);
                    }
                    break;
            }
        }
    }

    private void nextFollowerAction() {
        followerAction = followerActionList.poll();
    }

    private void customLaunchCloseShot() {
        if (!ballSensor.isDetected())
            intakeMotor.setIntakePanic(INTAKE_PANIC_TIME);
        launcher.launchCloseShot();
    }

    private void customLaunchCloseShot(int count, double interval) {
        for (int i = 0; i < count; i++) {
            auxTimer.resetTimer();
            do {} while (auxTimer.getElapsedTimeSeconds() < interval);
            customLaunchCloseShot();
        }
    }

    private void customLaunchFarShot() {
        if (!ballSensor.isDetected())
            intakeMotor.setIntakePanic(INTAKE_PANIC_TIME);
        launcher.launchFarShot();
    }

    private void customLaunchFarShot(int count, double interval) {
        for (int i = 0; i < count; i++) {
            auxTimer.resetTimer();
            do {} while (auxTimer.getElapsedTimeSeconds() < interval);
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