package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.geometry.Pose;

/**
 * This is the Follower's pre-configured position class. This class constructs and maintains all the
 * positions which are used in the game field. The positions are measured in game field practice
 * before they are configured or re-programmed. The FollowerPose contains all Blue and Red team game
 * usage position, including far and close start and stop positions. Depend on the situation, these
 * positions can be constructed by: FollowerPose followerPose = new FollowerPose("<blue|red>"); Then
 * calling the useFarStartPose() and useFarScorePose() methods to configure the game usage positions.
 *
 */
public class FollowerPose {
    private double x_start_pose_close, y_start_pose_close, h_start_pose_close;
    private double x_start_score_close, y_start_score_close, h_start_score_close;
    private double x_score_pose_close, y_score_pose_close, h_score_pose_close;
    private double x_stop_pose_close, y_stop_pose_close, h_stop_pose_close;
    private double x_start_pose_far, y_start_pose_far, h_start_pose_far;
    private double x_start_score_far, y_start_score_far, h_start_score_far;
    private double x_score_pose_far, y_score_pose_far, h_score_pose_far;
    private double x_stop_pose_far, y_stop_pose_far, h_stop_pose_far;
    private double x_high_spike_line_start, y_high_spike_line_start, h_high_spike_line_start;
    private double x_high_spike_line_end, y_high_spike_line_end, h_high_spike_line_end;
    private double x_high_spike_line_return, y_high_spike_line_return, h_high_spike_line_return;
    private double x_mid_spike_line_start, y_mid_spike_line_start, h_mid_spike_line_start;
    private double x_mid_spike_line_end, y_mid_spike_line_end, h_mid_spike_line_end;
    private double x_mid_spike_line_return, y_mid_spike_line_return, h_mid_spike_line_return;
    private double x_low_spike_line_start, y_low_spike_line_start, h_low_spike_line_start;
    private double x_low_spike_line_end, y_low_spike_line_end, h_low_spike_line_end;
    private double x_low_spike_line_return, y_low_spike_line_return, h_low_spike_line_return;
    private double x_gate_pose, y_gate_pose, h_gate_pose;
    private double x_gate_contact, y_gate_contact, h_gate_contact;
    private double x_gate_return, y_gate_return, h_gate_return;

    public Pose startPose, startScorePose, scorePose, stopPose,
            highSpkPose, highSpkEnd, highSpkReturn, midSpkPose, midSpkEnd, midSpkReturn,
            lowSpkPose, lowSpkEnd, lowSpkReturn, gatePose, gateContact, gateReturn;

    /** Set start position (x, y, heading): robot is over the score line, either touching the goal wall
     * or the field boundary.
     */
    public void setStartPose(double x, double y, double h) { startPose = new Pose(x, y, h); }

    /** Set start scoring position (x, y, heading): face the goal wall in preparing to score the
     * artifacts, only used after the starting position.
     */
    public void setStartScorePose(double x, double y, double h) { startScorePose = new Pose(x, y, h); }

    /** Set scoring position (x, y, heading): face the goal wall in preparing to score the artifacts.
     */
    public void setScorePose(double x, double y, double h) { scorePose = new Pose(x, y, h); }

    /** Set stop position (x, y, heading): at which the robot finishes its autonomous path.
     */
    public void setStopPose(double x, double y, double h) { stopPose = new Pose(x, y, h); }

    /** Set the start, end and return positions to grab the artifacts on highest spike mark.
     */
    public void setHighSpkPose(double x, double y, double h) { highSpkPose = new Pose(x, y, h); }
    public void setHighSpkEnd(double x, double y, double h) { highSpkEnd = new Pose(x, y, h); }
    public void setHighSpkReturn(double x, double y, double h) { highSpkReturn = new Pose(x, y, h); }

    /** Set the start, end and return positions to grab the artifacts on middle spike mark.
     */
    public void setMidSpkPose(double x, double y, double h) { midSpkPose = new Pose(x, y, h); }
    public void setMidSpkEnd(double x, double y, double h) { midSpkEnd = new Pose(x, y, h); }
    public void setMidSpkReturn(double x, double y, double h) { midSpkReturn = new Pose(x, y, h); }

    /** Set the start, end and return positions to grab the artifacts on lowest spike mark.
     */
    public void setLowSpkPose(double x, double y, double h) { lowSpkPose = new Pose(x, y, h); }
    public void setLowSpkEnd(double x, double y, double h) { lowSpkEnd = new Pose(x, y, h); }
    public void setLowSpkReturn(double x, double y, double h) { lowSpkReturn = new Pose(x, y, h); }

    /** Set the gate opening standby, contact positions to flex the gate open.
     */
    public void setGatePose(double x, double y, double h) { gatePose = new Pose(x, y, h); }
    public void setGateContact(double x, double y, double h) { gateContact = new Pose(x, y, h); }

    /** Set the gate return position after the gate opening action is done, and before proceeding to
     * the next position.
     */
    public void setGateReturn(double x, double y, double h) { gateReturn = new Pose(x, y, h); }

    /**
     * This is an empty constructor for the FollowerPose class so it can get started with the Blue
     * team default positions. The proper usage is after an empty constructor call:
     * FollowerPose followerPose = new FollowerPose(); Then using the useFarStartPose() and
     * useFarStopPose() methods to configure the game usage positions.
     */
    public FollowerPose() {
        this("blue");
    }

    /**
     * This is a constructor for the FollowerPose class so it can get started with the Blue or Red
     * team default positions. The proper usage is after an empty constructor call:
     * FollowerPose followerPose = new FollowerPose("<blue|red>"); Then using the useFarStartPose()
     * and useFarStopPose() methods to configure the game usage positions.
     */
    public FollowerPose(String string) {
        if (string.equalsIgnoreCase("blue"))
            defaultBluePose();
        else if (string.equalsIgnoreCase("red"))
            defaultRedPose();
        else
            throw new IllegalArgumentException("Invalid string argument");
    }

    /** The default Blue team positions, including far and close start and stop positions.
     * The positions are measured in game field practice, and any subsequent re-configured changes
     * must be re-measured and tested in the game filed before becoming the defaults.
     */
    public void defaultBluePose() {
        x_start_pose_close = 24.5;
        y_start_pose_close = 126.5;
        h_start_pose_close = 144.0;
        x_start_score_close = 55.0;
        y_start_score_close = 91.0;
        h_start_score_close = 142.0;
        x_score_pose_close = 55.0;
        y_score_pose_close = 91.0;
        h_score_pose_close = 142.0;
        x_stop_pose_close = 44.0;
        y_stop_pose_close = 69.0;
        h_stop_pose_close = 180.0;

        x_start_pose_far = 56.0;
        y_start_pose_far = 9.0;
        h_start_pose_far = 90.0;
        x_start_score_far = 56.0;
        y_start_score_far = 14.0;
        h_start_score_far = 116.0;
        x_score_pose_far = 56.0;
        y_score_pose_far = 15.0;
        h_score_pose_far = 124.5;
        x_stop_pose_far = 56.0;
        y_stop_pose_far = 33.0;
        h_stop_pose_far = 90.0;

        x_high_spike_line_start = 44.0;
        y_high_spike_line_start = 84.0;
        h_high_spike_line_start = 180.0;
        x_high_spike_line_end = 19.0;
        y_high_spike_line_end = y_high_spike_line_start;
        h_high_spike_line_end = h_high_spike_line_start;
        x_high_spike_line_return = x_high_spike_line_start;
        y_high_spike_line_return = y_high_spike_line_start;
        h_high_spike_line_return = h_high_spike_line_start - 20.0;

        x_mid_spike_line_start = 44.0;
        y_mid_spike_line_start = 60.0;
        h_mid_spike_line_start = 180.0;
        x_mid_spike_line_end = 19.0;
        y_mid_spike_line_end = y_mid_spike_line_start;
        h_mid_spike_line_end = h_mid_spike_line_start;
        x_mid_spike_line_return = x_mid_spike_line_start;
        y_mid_spike_line_return = y_mid_spike_line_start;
        h_mid_spike_line_return = h_mid_spike_line_start - 20.0;

        x_low_spike_line_start = 44.0;
        y_low_spike_line_start = 39.0; //36.0 1-4-26
        h_low_spike_line_start = 180.0;
        x_low_spike_line_end = 19.0;
        y_low_spike_line_end = y_low_spike_line_start;
        h_low_spike_line_end = h_low_spike_line_start;
        x_low_spike_line_return = x_low_spike_line_start;
        y_low_spike_line_return = y_low_spike_line_start;
        h_low_spike_line_return = h_low_spike_line_start - 20.0;

        x_gate_pose = 20.5;
        y_gate_pose = 69.0;
        h_gate_pose = 180.0;
        x_gate_contact = 18.0;
        y_gate_contact = 69.0;
        h_gate_contact = 180.0;
        x_gate_return = 44.0;
        y_gate_return = 69.0;
        h_gate_return = 160.0;

        setAllPose(); // set all game usage positions ready
    }

    /** The default Red team positions, including far and close start and stop positions.
     * The positions are measured in game field practice, and any subsequent re-configured changes
     * must be re-measured and tested in the game filed before becoming the defaults.
     */
    public void defaultRedPose() {
        x_start_pose_close = 119.5;
        y_start_pose_close = 126.5;
        h_start_pose_close = 24.0;
        x_start_score_close = 78.0; // 89.0; 1-7-26
        y_start_score_close = 95.0;  // 91.0; 1-7-26
        h_start_score_close = 34.0; // 38.0; 1-4-26
        x_score_pose_close = 78.0; // 89.0; 1-7-26
        y_score_pose_close = 95.0;  // 91.0; 1-7-26
        h_score_pose_close = 25.0; // 38.0; 1-4-26
        x_stop_pose_close = 78.0;
        y_stop_pose_close = 80.0;
        h_stop_pose_close = -12.0; // 180.0; 1-7-26

        x_start_pose_far = 88.0;
        y_start_pose_far = 9.0;
        h_start_pose_far = 87.0; // 90.0 1-4-26 (don't touch)
        x_start_score_far = 88.0;
        y_start_score_far = 14.0;
        h_start_score_far = 67.5; // 65.0 1-4-26 (don't touch)
        x_score_pose_far = 88.0;
        y_score_pose_far = 15.0;
        h_score_pose_far = 62.0; // 65.0 1-4-26 (don't touch)
        x_stop_pose_far = 88.0;
        y_stop_pose_far = 33.0;
        h_stop_pose_far = 90.0;

        x_high_spike_line_start = 89.0;  // 100.0 1-7-26
        y_high_spike_line_start = 86.5;  // 84.0 1-4-26
        h_high_spike_line_start = -12.0; // 0.0;
        x_high_spike_line_end = 113.0;
        y_high_spike_line_end = y_high_spike_line_start;
        h_high_spike_line_end = h_high_spike_line_start;
        x_high_spike_line_return = x_high_spike_line_start;
        y_high_spike_line_return = y_high_spike_line_start;
        h_high_spike_line_return = h_high_spike_line_start + 20.0;

        x_mid_spike_line_start = 85.0; // 100.0 1-7-26
        y_mid_spike_line_start = 65.0; // 60.0; 1-4-26
        h_mid_spike_line_start = h_high_spike_line_start;
        x_mid_spike_line_end = 111.5;
        y_mid_spike_line_end = y_mid_spike_line_start - 3.0;
        h_mid_spike_line_end = h_mid_spike_line_start - 5.0;
        x_mid_spike_line_return = x_mid_spike_line_start;
        y_mid_spike_line_return = y_mid_spike_line_start;
        h_mid_spike_line_return = h_mid_spike_line_start + 20.0;

        x_low_spike_line_start = 85.0; // 100.0
        y_low_spike_line_start = 35.5; // 36.0 1-4-26
        h_low_spike_line_start = h_high_spike_line_start;
        x_low_spike_line_end = 123.0;
        y_low_spike_line_end = y_low_spike_line_start;
        h_low_spike_line_end = h_low_spike_line_start;
        x_low_spike_line_return = x_low_spike_line_start;
        y_low_spike_line_return = y_low_spike_line_start;
        h_low_spike_line_return = h_low_spike_line_start + 20.0;

        x_gate_pose = 110.0;
        y_gate_pose = 69.0;
        h_gate_pose = -12.0; // 0.0;
        x_gate_contact = 111.5;
        y_gate_contact = 69.0;
        h_gate_contact = -12.0; // 0.0;
        x_gate_return = 100.0;
        y_gate_return = 69.0;
        h_gate_return = 5.0; // 20.0;

        setAllPose(); // set all game usage positions ready
    }

    /** Set all game usage positions ready by using the default x,y coordinates and heading values.
     */
    public void setAllPose() {
        setStartPose(x_start_pose_close, y_start_pose_close, Math.toRadians(h_start_pose_close));
        setStartScorePose(x_start_score_close, y_start_score_close, Math.toRadians(h_start_score_close));
        setScorePose(x_score_pose_close, y_score_pose_close, Math.toRadians(h_score_pose_close));
        setStopPose(x_stop_pose_close, y_stop_pose_close, Math.toRadians(h_stop_pose_close));

        setHighSpkPose(x_high_spike_line_start, y_high_spike_line_start, Math.toRadians(h_high_spike_line_start));
        setHighSpkEnd(x_high_spike_line_end, y_high_spike_line_end, Math.toRadians(h_high_spike_line_end));
        setHighSpkReturn(x_high_spike_line_return, y_high_spike_line_return, Math.toRadians(h_high_spike_line_return));

        setMidSpkPose(x_mid_spike_line_start, y_mid_spike_line_start, Math.toRadians(h_mid_spike_line_start));
        setMidSpkEnd(x_mid_spike_line_end, y_mid_spike_line_end, Math.toRadians(h_mid_spike_line_end));
        setMidSpkReturn(x_mid_spike_line_return, y_mid_spike_line_return, Math.toRadians(h_mid_spike_line_return));

        setLowSpkPose(x_low_spike_line_start, y_low_spike_line_start, Math.toRadians(h_low_spike_line_start));
        setLowSpkEnd(x_low_spike_line_end, y_low_spike_line_end, Math.toRadians(h_low_spike_line_end));
        setLowSpkReturn(x_low_spike_line_return, y_low_spike_line_return, Math.toRadians(h_low_spike_line_return));

        setGatePose(x_gate_pose, y_gate_pose, Math.toRadians(h_gate_pose));
        setGateContact(x_gate_contact, y_gate_contact, Math.toRadians(h_gate_contact));
        setGateReturn(x_gate_return, y_gate_return, Math.toRadians(h_gate_return));
    }

    /** Set to use the far start position.
     */
    public void useFarStartPose() {
        setStartPose(x_start_pose_far, y_start_pose_far, Math.toRadians(h_start_pose_far));
    }

    /** Set to use the far scoring position.
     */
    public void useFarScorePose() {
        setStartScorePose(x_start_score_far, y_start_score_far, Math.toRadians(h_start_score_far));
        setScorePose(x_score_pose_far, y_score_pose_far, Math.toRadians(h_score_pose_far));
    }

    /** Set to use the far stop position.
     */
    public void useFarStopPose() {
        setStopPose(x_stop_pose_far, y_stop_pose_far, Math.toRadians(h_stop_pose_far));
    }

    /** Set to use the close start position.
     */
    public void useCloseStartPose() {
        setStartPose(x_start_pose_close, y_start_pose_close, Math.toRadians(h_start_pose_close));
    }

    /** Set to use the close scoring position.
     */
    public void useCloseScorePose() {
        setStartScorePose(x_start_score_close, y_start_score_close, Math.toRadians(h_start_score_close));
        setScorePose(x_score_pose_close, y_score_pose_close, Math.toRadians(h_score_pose_close));
    }

    /** Set to use the close stop position.
     */
    public void useCloseStopPose() {
        setStopPose(x_stop_pose_close, y_stop_pose_close, Math.toRadians(h_stop_pose_close));
    }
}