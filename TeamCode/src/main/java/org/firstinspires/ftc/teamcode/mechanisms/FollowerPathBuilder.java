package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

public class FollowerPathBuilder {
    private final Follower follower;
    private final FollowerPose followerPose;

    public FollowerPathBuilder(Follower follower, FollowerPose followerPose, PathConstraints constraints) {
        this.follower = follower;
        this.followerPose = followerPose;
        PathConstraints.setDefaultConstraints(constraints);
    }

    public FollowerPathBuilder(Follower follower, FollowerPose followerPose) {
        this(follower, followerPose, PathConstraints.defaultConstraints);
    }

    public PathChain buildPaths_startPos2Score() {
        return follower.pathBuilder() // move from start position to scoring position
                .addPath(new BezierLine(followerPose.startPose, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(), followerPose.scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2Stop() {
        return follower.pathBuilder() // move from start position to stop position
                .addPath(new BezierLine(followerPose.startPose, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(), followerPose.stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2Gate() {
        return follower.pathBuilder() // move from start position to gate open standby position
                .addPath(new BezierLine(followerPose.startPose, followerPose.gatePoseUp))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(), followerPose.gatePoseUp.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2HighSpk() {
        return follower.pathBuilder() // move from start position to high spike mark position
                .addPath(new BezierLine(followerPose.startPose, followerPose.highSpkPose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(), followerPose.highSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2MidSpk() {
        return follower.pathBuilder() // move from start position to middle spike mark position
                .addPath(new BezierLine(followerPose.startPose, followerPose.midSpkPose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(), followerPose.midSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_startPos2LowSpk() {
        return follower.pathBuilder() // move from start position to low spike mark position
                .addPath(new BezierLine(followerPose.startPose, followerPose.lowSpkPose))
                .setLinearHeadingInterpolation(followerPose.startPose.getHeading(), followerPose.lowSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2Stop() {
        return follower.pathBuilder()// move from scoring position to stop position
                .addPath(new BezierLine(followerPose.scorePose, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(), followerPose.stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2Gate() {
        return follower.pathBuilder()// move from scoring position to gate open standby position
                .addPath(new BezierLine(followerPose.scorePose, followerPose.gatePoseUp))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(), followerPose.gatePoseUp.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2HighSpk() {
        return follower.pathBuilder() // move from scoring position to highest spike mark position
                .addPath(new BezierLine(followerPose.scorePose, followerPose.highSpkPose))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(), followerPose.highSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2MidSpk() {
        return follower.pathBuilder() // move from scoring position to middle spike mark position
                .addPath(new BezierLine(followerPose.scorePose, followerPose.midSpkPose))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(), followerPose.midSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_scorePos2LowSpk() {
        return follower.pathBuilder() // move from scoring position to lowest spike mark position
                .addPath(new BezierLine(followerPose.scorePose, followerPose.lowSpkPose))
                .setLinearHeadingInterpolation(followerPose.scorePose.getHeading(), followerPose.lowSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_highSpkPos2GrabEnd() {
        return follower.pathBuilder() // grab the highest spike mark artifacts
                .addPath(new BezierLine(followerPose.highSpkPose, followerPose.highSpkEnd))
                .setLinearHeadingInterpolation(followerPose.highSpkPose.getHeading(), followerPose.highSpkEnd.getHeading())
                .build();
    }

    public PathChain buildPaths_midSpkPos2GrabEnd() {
        return follower.pathBuilder() // grab the middle spike mark artifacts
                .addPath(new BezierLine(followerPose.midSpkPose, followerPose.midSpkEnd))
                .setLinearHeadingInterpolation(followerPose.midSpkPose.getHeading(), followerPose.midSpkEnd.getHeading())
                .build();
    }

    public PathChain buildPaths_lowSpkPos2GrabEnd() {
        return follower.pathBuilder() // grab the lowest spike mark artifacts
                .addPath(new BezierLine(followerPose.lowSpkPose, followerPose.lowSpkEnd))
                .setLinearHeadingInterpolation(followerPose.lowSpkPose.getHeading(), followerPose.lowSpkEnd.getHeading())
                .build();
    }

    public PathChain buildPaths_highSpkEnd2Score() {
        return follower.pathBuilder() // move back to scoring position
                .addPath(new BezierLine(followerPose.highSpkEnd, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.highSpkEnd.getHeading(), followerPose.scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_midSpkEnd2Score() {
        return follower.pathBuilder() // move back to scoring position
                .addPath(new BezierLine(followerPose.midSpkEnd, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.midSpkEnd.getHeading(), followerPose.scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_lowSpkEnd2Score() {
        return follower.pathBuilder() // move back to scoring position
                .addPath(new BezierLine(followerPose.lowSpkEnd, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.lowSpkEnd.getHeading(), followerPose.scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_highSpkEnd2Gate() {
        return follower.pathBuilder() // move to gate open standby position
                .addPath(new BezierLine(followerPose.highSpkEnd, followerPose.gatePoseUp))
                .setLinearHeadingInterpolation(followerPose.highSpkEnd.getHeading(), followerPose.gatePoseUp.getHeading())
                .build();
    }

    public PathChain buildPaths_midSpkEnd2Gate() {
        return follower.pathBuilder() // move to gate open standby position
                .addPath(new BezierLine(followerPose.midSpkEnd, followerPose.gatePoseDw))
                .setLinearHeadingInterpolation(followerPose.midSpkEnd.getHeading(), followerPose.gatePoseDw.getHeading())
                .build();
    }

    public PathChain buildPaths_lowSpkEnd2Gate() {
        return follower.pathBuilder() // move to gate open standby position
                .addPath(new BezierLine(followerPose.lowSpkEnd, followerPose.gatePoseDw))
                .setLinearHeadingInterpolation(followerPose.lowSpkEnd.getHeading(), followerPose.gatePoseDw.getHeading())
                .build();
    }

    public PathChain buildPaths_highSpkEnd2Stop() {
        return follower.pathBuilder() // move to stop position
                .addPath(new BezierLine(followerPose.highSpkEnd, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.highSpkEnd.getHeading(), followerPose.stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_midSpkEnd2Stop() {
        return follower.pathBuilder() // move to stop position
                .addPath(new BezierLine(followerPose.midSpkEnd, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.midSpkEnd.getHeading(), followerPose.stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_lowSpkEnd2Stop() {
        return follower.pathBuilder() // move to stop position
                .addPath(new BezierLine(followerPose.lowSpkEnd, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.lowSpkEnd.getHeading(), followerPose.stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_gatePosUp2Contact() {
        return follower.pathBuilder() // move from gate open standby position to contact position
                .addPath(new BezierLine(followerPose.gatePoseUp, followerPose.gateContactUp))
                .setLinearHeadingInterpolation(followerPose.gatePoseUp.getHeading(), followerPose.gateContactUp.getHeading())
                .build();
    }

    public PathChain buildPaths_gatePosDw2Contact() {
        return follower.pathBuilder() // move from gate open standby position to contact position
                .addPath(new BezierLine(followerPose.gatePoseDw, followerPose.gateContactDw))
                .setLinearHeadingInterpolation(followerPose.gatePoseDw.getHeading(), followerPose.gateContactDw.getHeading())
                .build();
    }

    public PathChain buildPaths_gateContactUp2Return() {
        return follower.pathBuilder() // move from contact position to gate return position
                .addPath(new BezierLine(followerPose.gateContactUp, followerPose.gateReturn))
                .setLinearHeadingInterpolation(followerPose.gateContactUp.getHeading(), followerPose.gateReturn.getHeading())
                .build();
    }

    public PathChain buildPaths_gateContactDw2Return() {
        return follower.pathBuilder() // move from contact position to gate return position
                .addPath(new BezierLine(followerPose.gateContactDw, followerPose.gateReturn))
                .setLinearHeadingInterpolation(followerPose.gateContactDw.getHeading(), followerPose.gateReturn.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturn2Score() {
        return follower.pathBuilder() // move from gate return position to scoring position
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.scorePose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(), followerPose.scorePose.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturnPos2Stop() {
        return follower.pathBuilder() // move from gate return position to stop position
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.stopPose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(), followerPose.stopPose.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturnPos2HighSpk() {
        return follower.pathBuilder() // move from gate return position to highest spike mark position
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.highSpkPose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(), followerPose.highSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturnPos2MidSpk() {
        return follower.pathBuilder() // move from gate return position to middle spike mark position
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.midSpkPose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(), followerPose.midSpkPose.getHeading())
                .build();
    }

    public PathChain buildPaths_gateReturnPos2LowSpk() {
        return follower.pathBuilder() // move from gate return position to lowest spike mark position
                .addPath(new BezierLine(followerPose.gateReturn, followerPose.lowSpkPose))
                .setLinearHeadingInterpolation(followerPose.gateReturn.getHeading(), followerPose.lowSpkPose.getHeading())
                .build();
    }
}
