package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.geometry.Pose;

public class FollowerPose {
    private double x_start_pose_close, y_start_pose_close, h_start_pose_close;
    private double x_score_pose_close, y_score_pose_close, h_score_pose_close;
    private double x_stop_pose_close, y_stop_pose_close, h_stop_pose_close;
    private double x_start_pose_far, y_start_pose_far, h_start_pose_far;
    private double x_score_pose_far, y_score_pose_far, h_score_pose_far;
    private double x_stop_pose_far, y_stop_pose_far, h_stop_pose_far;
    private double x_high_spike_line_start, y_high_spike_line_start, h_high_spike_line_start;
    private double x_high_spike_line_end, y_high_spike_line_end, h_high_spike_line_end;
    private double x_mid_spike_line_start, y_mid_spike_line_start, h_mid_spike_line_start;
    private double x_mid_spike_line_end, y_mid_spike_line_end, h_mid_spike_line_end;
    private double x_low_spike_line_start, y_low_spike_line_start, h_low_spike_line_start;
    private double x_low_spike_line_end, y_low_spike_line_end, h_low_spike_line_end;
    private double x_gate_pose, y_gate_pose, h_gate_pose_up, h_gate_pose_dw;
    private double x_gate_contact, y_gate_contact, h_gate_contact_up, h_gate_contact_dw;
    private double x_gate_return, y_gate_return, h_gate_return;

    public Pose startPose, scorePose, stopPose, highSpkPose, highSpkEnd, midSpkPose, midSpkEnd,
            lowSpkPose, lowSpkEnd, gatePoseUp, gatePoseDw, gateContactUp, gateContactDw, gateReturn;

    // start pose. robot is over the score line, either touching the goal wall or field boundary.
    public void setStartPose(double x, double y, double h) { startPose = new Pose(x, y, h); }

    // stop pose at which the robot finishes its autonomous path.
    public void setStopPose(double x, double y, double h) { stopPose = new Pose(x, y, h); }

    // scoring pose. facing the goal wall to score the artifacts.
    public void setScorePose(double x, double y, double h) { scorePose = new Pose(x, y, h); }

    // grab pose of the highest (1st set) artifacts spike mark.
    public void setHighSpkPose(double x, double y, double h) { highSpkPose = new Pose(x, y, h); }
    public void setHighSpkEnd(double x, double y, double h) { highSpkEnd = new Pose(x, y, h); }

    // grab pose of the middle (2nd set) artifacts spike mark.
    public void setMidSpkPose(double x, double y, double h) { midSpkPose = new Pose(x, y, h); }
    public void setMidSpkEnd(double x, double y, double h) { midSpkEnd = new Pose(x, y, h); }

    // grab pose of the lowest (3rd set) artifacts spike mark.
    public void setLowSpkPose(double x, double y, double h) { lowSpkPose = new Pose(x, y, h); }
    public void setLowSpkEnd(double x, double y, double h) { lowSpkEnd = new Pose(x, y, h); }

    // gate start and return poses to flex the gate open.
    public void setGatePoseUp(double x, double y, double h) { gatePoseUp = new Pose(x, y, h); }
    public void setGatePoseDw(double x, double y, double h) { gatePoseDw = new Pose(x, y, h); }
    public void setGateContactUp(double x, double y, double h) { gateContactUp = new Pose(x, y, h); }
    public void setGateContactDw(double x, double y, double h) { gateContactDw = new Pose(x, y, h); }
    public void setGateReturn(double x, double y, double h) { gateReturn = new Pose(x, y, h); }

    private boolean useRedPose, useFarStartPose, useFarStopPose;

    public FollowerPose() {
        this("blue");
    }

    public FollowerPose(String string) {
        if (string.equalsIgnoreCase("blue")) {
            useRedPose = false;
            useFarStartPose = false;
            useFarStopPose = false;
            create();
        } else if (string.equalsIgnoreCase("red")) {
            useRedPose = true;
            useFarStartPose = false;
            useFarStopPose = false;
            create();
        } else
            throw new IllegalArgumentException("Invalid string argument");
    }

    public FollowerPose create() {
        if (useRedPose)
            defaultRedPose();
        else
            defaultBluePose();

        if (useFarStartPose)
            useFarStartPose();
        else
            useCloseStartPose();

        if (useFarStopPose)
            useFarStopPose();
        else
            useCloseStopPose();

        return this;
    }

    public FollowerPose useRedPose(boolean useRedPose) {
        this.useRedPose = useRedPose;
        return this;
    }

    public FollowerPose useFarStartPose(boolean useFarStartPose) {
        this.useFarStartPose = useFarStartPose;
        return this;
    }

    public FollowerPose useFarStopPose(boolean useFarStopPose) {
        this.useFarStopPose = useFarStopPose;
        return this;
    }

    public void defaultBluePose() {
        x_start_pose_close = 24.5;
        y_start_pose_close = 126.5;
        h_start_pose_close = 144.0;
        x_score_pose_close = 55.0;
        y_score_pose_close = 91.0;
        h_score_pose_close = 140.0;
        x_stop_pose_close = 55.0;
        y_stop_pose_close = 69.0;
        h_stop_pose_close = 90.0;

        x_start_pose_far = 56.0;
        y_start_pose_far = 9.0;
        h_start_pose_far = 90.0;
        x_score_pose_far = 56.0;
        y_score_pose_far = 12.0;
        h_score_pose_far = 115.0;
        x_stop_pose_far = 56.0;
        y_stop_pose_far = 33.0;
        h_stop_pose_far = 90.0;

        x_high_spike_line_start = 44.0;
        y_high_spike_line_start = 84.0;
        h_high_spike_line_start = 180.0;
        x_high_spike_line_end = 17.5;
        y_high_spike_line_end = 84.0;
        h_high_spike_line_end = 180.0;

        x_mid_spike_line_start = 44.0;
        y_mid_spike_line_start = 60.0;
        h_mid_spike_line_start = 180.0;
        x_mid_spike_line_end = 16.5;
        y_mid_spike_line_end = 60.0;
        h_mid_spike_line_end = 180.0;

        x_low_spike_line_start = 44.0;
        y_low_spike_line_start = 36.0;
        h_low_spike_line_start = 180.0;
        x_low_spike_line_end = 16.5;
        y_low_spike_line_end = 36.0;
        h_low_spike_line_end = 180.0;

        x_gate_pose = 17.5;
        y_gate_pose = 69.0;
        h_gate_pose_up = 90.0;
        h_gate_pose_dw = -90.0;
        x_gate_contact = 14;
        y_gate_contact = 69.0;
        h_gate_contact_up = 90.0;
        h_gate_contact_dw = -90.0;
        x_gate_return = 44.0;
        y_gate_return = 69.0;
        h_gate_return = 20.0;

        setAllPose();
    }

    public void defaultRedPose() {
        x_start_pose_close = 119.5;
        y_start_pose_close = 126.5;
        h_start_pose_close = 36.0;
        x_score_pose_close = 89.0;
        y_score_pose_close = 91.0;
        h_score_pose_close = 40.0;
        x_stop_pose_close = 89.0;
        y_stop_pose_close = 69.0;
        h_stop_pose_close = 90.0;

        x_start_pose_far = 88.0;
        y_start_pose_far = 9.0;
        h_start_pose_far = 90.0;
        x_score_pose_far = 88.0;
        y_score_pose_far = 12.0;
        h_score_pose_far = 65.0;
        x_stop_pose_far = 88.0;
        y_stop_pose_far = 33.0;
        h_stop_pose_far = 90.0;

        x_high_spike_line_start = 100.0;
        y_high_spike_line_start = 84.0;
        h_high_spike_line_start = 0.0;
        x_high_spike_line_end = 126.5;
        y_high_spike_line_end = 84.0;
        h_high_spike_line_end = 0.0;

        x_mid_spike_line_start = 100.0;
        y_mid_spike_line_start = 60.0;
        h_mid_spike_line_start = 0.0;
        x_mid_spike_line_end = 127.5;
        y_mid_spike_line_end = 60.0;
        h_mid_spike_line_end = 0.0;

        x_low_spike_line_start = 100.0;
        y_low_spike_line_start = 36.0;
        h_low_spike_line_start = 0.0;
        x_low_spike_line_end = 127.5;
        y_low_spike_line_end = 36.0;
        h_low_spike_line_end = 0.0;

        x_gate_pose = 126.5;
        y_gate_pose = 69.0;
        h_gate_pose_up = 90.0;
        h_gate_pose_dw = -90.0;
        x_gate_contact = 130;
        y_gate_contact = 69.0;
        h_gate_contact_up = 90.0;
        h_gate_contact_dw = -90.0;
        x_gate_return = 100.0;
        y_gate_return = 69.0;
        h_gate_return = 160.0;

        setAllPose();
    }

    public void setAllPose() {
        setStartPose(x_start_pose_close, y_start_pose_close, Math.toRadians(h_start_pose_close));
        setScorePose(x_score_pose_close, y_score_pose_close, Math.toRadians(h_score_pose_close));
        setStopPose(x_stop_pose_close, y_stop_pose_close, Math.toRadians(h_stop_pose_close));
        setHighSpkPose(x_high_spike_line_start, y_high_spike_line_start, Math.toRadians(h_high_spike_line_start));
        setMidSpkPose(x_mid_spike_line_start, y_mid_spike_line_start, Math.toRadians(h_mid_spike_line_start));
        setLowSpkPose(x_low_spike_line_start, y_low_spike_line_start, Math.toRadians(h_low_spike_line_start));
        setHighSpkEnd(x_high_spike_line_end, y_high_spike_line_end, Math.toRadians(h_high_spike_line_end));
        setMidSpkEnd(x_mid_spike_line_end, y_mid_spike_line_end, Math.toRadians(h_mid_spike_line_end));
        setLowSpkEnd(x_low_spike_line_end, y_low_spike_line_end, Math.toRadians(h_low_spike_line_end));
        setGatePoseUp(x_gate_pose, y_gate_pose, Math.toRadians(h_gate_pose_up));
        setGatePoseDw(x_gate_pose, y_gate_pose, Math.toRadians(h_gate_pose_dw));
        setGateContactUp(x_gate_contact, y_gate_contact, Math.toRadians(h_gate_contact_up));
        setGateContactDw(x_gate_contact, y_gate_contact, Math.toRadians(h_gate_contact_dw));
        setGateReturn(x_gate_return, y_gate_return, Math.toRadians(h_gate_return));
    }

    public void useFarStartPose() {
        setStartPose(x_start_pose_far, y_start_pose_far, Math.toRadians(h_start_pose_far));
    }

    public void useFarScorePose() {
        setScorePose(x_score_pose_far, y_score_pose_far, Math.toRadians(h_score_pose_far));
    }

    public void useFarStopPose() {
        setStopPose(x_stop_pose_far, y_stop_pose_far, Math.toRadians(h_stop_pose_far));
    }

    public void useCloseStartPose() {
        setStartPose(x_start_pose_close, y_start_pose_close, Math.toRadians(h_start_pose_close));
    }

    public void useCloseScorePose() {
        setScorePose(x_score_pose_close, y_score_pose_close, Math.toRadians(h_score_pose_close));
    }

    public void useCloseStopPose() {
        setStopPose(x_stop_pose_close, y_stop_pose_close, Math.toRadians(h_stop_pose_close));
    }
}