package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

/**
 * This is the Follower's PathBuilder class. This class returns a specific PathChain on the fly
 * to the Follower through one of the buildPaths() method from the Follower's pre-configured
 * positions.
 *
 */
public class FollowerPathBuilder {
    private final Follower follower;
    private final FollowerPose followerPose;

    /**
     * This is a constructor for the Follower's PathBuilder class with the pre-configured
     * positions and path constraints as input arguments. The proper usage is using an instance of
     * the Follower class: Follower follower = new Follower(hardwareMap), and the other instance of
     * the FollowerPose class: FollowerPose followerPose = new FollowerPose(); Then calling the
     * PathBuilder() constructor. The path constraints are applied to all the generated pathChains
     * by default if not specified again in the individual buildPaths() method.
     *
     */
    public FollowerPathBuilder(Follower follower, FollowerPose followerPose,
                               PathConstraints constraints) {
        this.follower = follower;
        this.followerPose = followerPose;
        PathConstraints.setDefaultConstraints(constraints);
    }

    /**
     * This is a constructor for the Follower's PathBuilder class with the pre-configured
     * positions and path constraints as input arguments. The proper usage is using an instance of
     * the Follower class: Follower follower = new Follower(hardwareMap), and the other instance of
     * the FollowerPose class: FollowerPose followerPose = new FollowerPose(); Then calling the
     * PathBuilder() constructor. The default path constraints in the PathConstraints constants will
     * be used and applied to all the generate pathChains in the buildPaths() method.
     *
     */
    public FollowerPathBuilder(Follower follower, FollowerPose followerPose) {
        this(follower, followerPose, PathConstraints.defaultConstraints);
    }

    /**
     * Returns: PathChain from the Follower's start position to scoring position. The Follower will
     * face perpendicular to the goal at finish in preparing to shoot the artifacts.
     */
    public PathChain buildPaths_startPos2Score() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.startPose, followerPose.startScorePose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(),
                        followerPose.startScorePose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's start position to stop position.
     */
    public PathChain buildPaths_startPos2Stop() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.startPose, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(),
                        followerPose.stopPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's start position to gate opening standby position.
     * The Follower will have a 90 degrees heading in the field (heading away from audience) at
     * finish.
     */
    public PathChain buildPaths_startPos2Gate() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.startPose, followerPose.gatePoseUp))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(),
                        followerPose.gatePoseUp.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's start position to the highest spark mark position.
     * The highest spark mark position is the furthest spike mark away from the audience.
     * The Follower will face perpendicular to the gate wall at finish.
     */
    public PathChain buildPaths_startPos2HighSpk() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.startPose, followerPose.highSpkPose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(),
                        followerPose.highSpkPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's start position to the middle spark mark position.
     * The middle spark mark position is the 2nd furthest spike mark away from the audience.
     * The Follower will face perpendicular to the gate wall at finish.
     */
    public PathChain buildPaths_startPos2MidSpk() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.startPose, followerPose.midSpkPose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(),
                        followerPose.midSpkPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's start position to the lowest spark mark position.
     * The lowest spark mark position is the closest spike mark from the audience. The Follower
     * will face perpendicular to the gate wall at finish.
     */
    public PathChain buildPaths_startPos2LowSpk() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.startPose, followerPose.lowSpkPose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(),
                        followerPose.lowSpkPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's scoring position to the stop position.
     */
    public PathChain buildPaths_scorePos2Stop() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.scorePose, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(),
                        followerPose.stopPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's scoring position to gate opening standby position.
     * The Follower will have a 90 degrees heading in the field (heading away from audience) at
     * finish.
     */
    public PathChain buildPaths_scorePos2Gate() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.scorePose, followerPose.gatePoseUp))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(),
                        followerPose.gatePoseUp.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's scoring position to the highest spark mark position.
     * The highest spark mark position is the furthest spike mark away from the audience.
     * The Follower will face perpendicular to the gate wall at finish.
     */
    public PathChain buildPaths_scorePos2HighSpk() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.scorePose, followerPose.highSpkPose))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(),
                        followerPose.highSpkPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's scoring position to the middle spark mark position.
     * The middle spark mark position is the 2nd furthest spike mark away from the audience.
     * The Follower will face perpendicular to the gate wall at finish.
     */
    public PathChain buildPaths_scorePos2MidSpk() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.scorePose, followerPose.midSpkPose))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(),
                        followerPose.midSpkPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's scoring position to the lowest spark mark position.
     * The lowest spark mark position is the closest spike mark from the audience. The Follower
     * will face perpendicular to the gate wall at finish.
     */
    public PathChain buildPaths_scorePos2LowSpk() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.scorePose, followerPose.lowSpkPose))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(),
                        followerPose.lowSpkPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's highest spark mark start position to the end position.
     * The Follower is expected to intake all the artifacts on the ground at the spark mark.
     */
    public PathChain buildPaths_highSpkPos2GrabEnd() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.highSpkPose, followerPose.highSpkEnd))
                .setLinearHeadingInterpolation(followerPose.highSpkPose.getHeading(),
                        followerPose.highSpkEnd.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's middle spark mark start position to the end position.
     * The Follower is expected to intake all the artifacts on the ground at the spark mark.
     */
    public PathChain buildPaths_midSpkPos2GrabEnd() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.midSpkPose, followerPose.midSpkEnd))
                .setLinearHeadingInterpolation(followerPose.midSpkPose.getHeading(),
                        followerPose.midSpkEnd.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's lowest spark mark start position to the end position.
     * The Follower is expected to intake all the artifacts on the ground at the spark mark.
     */
    public PathChain buildPaths_lowSpkPos2GrabEnd() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.lowSpkPose, followerPose.lowSpkEnd))
                .setLinearHeadingInterpolation(followerPose.lowSpkPose.getHeading(),
                        followerPose.lowSpkEnd.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's highest spark mark end position to the scoring position.
     */
    public PathChain buildPaths_highSpkEnd2Score() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.highSpkEnd, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.highSpkEnd.getHeading(),
                        followerPose.scorePose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's highest spark mark end position back to the start
     * position before proceeding to the scoring position.
     */
    public PathChain buildPaths_highSpkEnd2Return() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.highSpkEnd, followerPose.highSpkReturn))
                .setLinearHeadingInterpolation(followerPose.highSpkEnd.getHeading(),
                        followerPose.highSpkReturn.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's highest spark mark return position back to the score
     * position.
     */
    public PathChain buildPaths_highSpkReturn2Score() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.highSpkReturn, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.highSpkReturn.getHeading(),
                        followerPose.scorePose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's middle spark mark end position to the scoring position.
     */
    public PathChain buildPaths_midSpkEnd2Score() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.midSpkEnd, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.midSpkEnd.getHeading(),
                        followerPose.scorePose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's middle spark mark end position back to the start
     * position before proceeding to the scoring position.
     */
    public PathChain buildPaths_midSpkEnd2Return() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.midSpkEnd, followerPose.midSpkReturn))
                .setLinearHeadingInterpolation(followerPose.midSpkEnd.getHeading(),
                        followerPose.midSpkReturn.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's middle spark mark return position back to the score
     * position.
     */
    public PathChain buildPaths_midSpkReturn2Score() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.midSpkReturn, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.midSpkReturn.getHeading(),
                        followerPose.scorePose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's lowest spark mark end position to the scoring position.
     */
    public PathChain buildPaths_lowSpkEnd2Score() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.lowSpkEnd, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.lowSpkEnd.getHeading(),
                        followerPose.scorePose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's lowest spark mark end position back to the start
     * position before proceeding to the scoring position.
     */
    public PathChain buildPaths_lowSpkEnd2Return() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.lowSpkEnd, followerPose.lowSpkReturn))
                .setLinearHeadingInterpolation(followerPose.lowSpkEnd.getHeading(),
                        followerPose.lowSpkReturn.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's lowest spark mark return position back to the score
     * position.
     */
    public PathChain buildPaths_lowSpkReturn2Score() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.lowSpkReturn, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.lowSpkReturn.getHeading(),
                        followerPose.scorePose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's highest spark mark end position to gate opening
     * standby position. The Follower will have a 90 degrees heading in the field (heading away
     * from audience) at finish.
     */
    public PathChain buildPaths_highSpkEnd2Gate() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.highSpkEnd, followerPose.gatePoseUp))
                .setLinearHeadingInterpolation(followerPose.highSpkEnd.getHeading(),
                        followerPose.gatePoseUp.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's middle spark mark end position to gate opening
     * standby position. The Follower will have a -90 degrees heading in the field
     * (heading towards the audience) at finish.
     */
    public PathChain buildPaths_midSpkEnd2Gate() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.midSpkEnd, followerPose.gatePoseDw))
                .setLinearHeadingInterpolation(followerPose.midSpkEnd.getHeading(),
                        followerPose.gatePoseDw.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's lowest spark mark end position to gate opening
     * standby position. The Follower will have a -90 degrees heading in the field
     * (heading towards the audience) at finish.
     */
    public PathChain buildPaths_lowSpkEnd2Gate() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.lowSpkEnd, followerPose.gatePoseDw))
                .setLinearHeadingInterpolation(followerPose.lowSpkEnd.getHeading(),
                        followerPose.gatePoseDw.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's highest spark mark end position to stop position.
     */
    public PathChain buildPaths_highSpkEnd2Stop() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.highSpkEnd, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.highSpkEnd.getHeading(),
                        followerPose.stopPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's middle spark mark end position to stop position.
     */
    public PathChain buildPaths_midSpkEnd2Stop() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.midSpkEnd, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.midSpkEnd.getHeading(),
                        followerPose.stopPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's lowest spark mark end position to stop position.
     */
    public PathChain buildPaths_lowSpkEnd2Stop() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.lowSpkEnd, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.lowSpkEnd.getHeading(),
                        followerPose.stopPose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's gate opening standby position to contact position.
     * The Follower will have a 90 degrees heading in the field (heading away from audience) at
     * the gate contact position. The gate is expected to get opened and the artifacts will be
     * released from the ramp.
     */
    public PathChain buildPaths_gatePosUp2Contact() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.gatePoseUp, followerPose.gateContactUp))
                .setLinearHeadingInterpolation(followerPose.gatePoseUp.getHeading(),
                        followerPose.gateContactUp.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's gate opening standby position to contact position.
     * The Follower will have a -90 degrees heading in the field (heading towards the audience) at
     * the gate contact position. The gate is expected to get opened and the artifacts will be
     * released from the ramp.
     */
    public PathChain buildPaths_gatePosDw2Contact() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.gatePoseDw, followerPose.gateContactDw))
                .setLinearHeadingInterpolation(followerPose.gatePoseDw.getHeading(),
                        followerPose.gateContactDw.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's gate contact position to return to an intermediate
     * position before proceeding to the scoring position. The Follower has a 90 degrees heading
     * in the field (heading away from audience) at the gate contact position.
     */
    public PathChain buildPaths_gateContactUp2Return() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.gateContactUp, followerPose.gateReturn))
                .setLinearHeadingInterpolation(followerPose.gateContactUp.getHeading(),
                        followerPose.gateReturn.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's gate contact position to return to an intermediate
     * position before proceeding to the scoring position. The Follower has a -90 degrees heading
     * in the field (heading towards the audience) at the gate contact position.
     */
    public PathChain buildPaths_gateContactDw2Return() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.gateContactDw, followerPose.gateReturn))
                .setLinearHeadingInterpolation(followerPose.gateContactDw.getHeading(),
                        followerPose.gateReturn.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's gate return position to the scoring position.
     */
    public PathChain buildPaths_gateReturn2Score() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(),
                        followerPose.scorePose.getHeading())
                .build();
    }

    /**
     * Returns: PathChain from the Follower's gate return position to the stop position.
     */
    public PathChain buildPaths_gateReturnPos2Stop() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(),
                        followerPose.stopPose.getHeading())
                .build();
    }

    /**
     * PathChain from the Follower's gate return position to the highest spark mark position.
     * The highest spark mark position is the furthest spike mark away from the audience.
     * The Follower will face perpendicular to the gate wall at finish.
     */
    public PathChain buildPaths_gateReturnPos2HighSpk() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.highSpkPose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(),
                        followerPose.highSpkPose.getHeading())
                .build();
    }

    /**
     * PathChain from the Follower's gate return position to the middle spark mark position.
     * The middle spark mark position is the 2nd furthest spike mark away from the audience.
     * The Follower will face perpendicular to the gate wall at finish.
     */
    public PathChain buildPaths_gateReturnPos2MidSpk() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.midSpkPose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(),
                        followerPose.midSpkPose.getHeading())
                .build();
    }

    /**
     * PathChain from the Follower's gate return position to the lowest spark mark position.
     * The lowest spark mark position is the closest spike mark from the audience. The Follower
     * will face perpendicular to the gate wall at finish.
     */
    public PathChain buildPaths_gateReturnPos2LowSpk() {
        return follower.pathBuilder()
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.lowSpkPose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(),
                        followerPose.lowSpkPose.getHeading())
                .build();
    }
}