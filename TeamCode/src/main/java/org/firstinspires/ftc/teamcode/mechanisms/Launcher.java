package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private DcMotorEx launcher;

    private String leftFeederName = "leftFeeder";
    private String rightFeederName = "rightFeeder";
    private String launcherName = "launcher";

    DcMotorSimple.Direction leftFeederDirection = FORWARD;
    DcMotorSimple.Direction rightFeederDirection = REVERSE;
    DcMotorSimple.Direction launcherDirection = FORWARD;
    DcMotor.ZeroPowerBehavior laucherZeroPowerBehavior = BRAKE;

    private double closeTargetVelocity = 1300;
    private double closeMinVelocity    = 1275;
    private double farTargetVelocity   = 1600;
    private double farMinVelocity      = 1575;

    private double feederRunSec = 0.10;
    private double launcherCoolOffSec = 0.20;

    // === Launcher state machine ===
    private enum LaunchState { IDLE, PANIC, SPIN_UP_F, SPIN_UP_C, LAUNCH, LAUNCHING, COOL_OFF }
    private LaunchState launchState = LaunchState.IDLE;
    private boolean isBusy = false;

    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime launchTimer = new ElapsedTime();

    public void build(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, launcherName);
        launcher.setMode(RUN_USING_ENCODER);
        launcher.setDirection(launcherDirection);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setVelocity(0.0);
        launcher.setPIDFCoefficients(RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));
        leftFeeder = hardwareMap.get(CRServo.class, leftFeederName);
        rightFeeder = hardwareMap.get(CRServo.class, rightFeederName);
        leftFeeder.setDirection(leftFeederDirection);
        rightFeeder.setDirection(rightFeederDirection);
        leftFeeder.setPower(0.0);
        rightFeeder.setPower(0.0);
    }

    public Launcher launcherName(String launcherName) {
        this.launcherName = launcherName;
        return this;
    }
    public Launcher leftFeederName(String feederName) {
        this.leftFeederName = feederName;
        return this;
    }

    public Launcher rightFeederName(String feederName) {
        this.rightFeederName = feederName;
        return this;
    }

    public Launcher launcherDirection(DcMotorSimple.Direction motorDirection) {
        launcherDirection = motorDirection;
        return this;
    }

    public Launcher launcherUseBrakeMode(boolean useBrakeMode) {
        if (useBrakeMode)
            laucherZeroPowerBehavior = BRAKE;
        else
            laucherZeroPowerBehavior = FLOAT;
        return this;
    }

    public Launcher leftFeederDirection(DcMotorSimple.Direction motorDirection) {
        leftFeederDirection = motorDirection;
        return this;
    }

    public Launcher rightFeederDirection(DcMotorSimple.Direction motorDirection) {
        rightFeederDirection = motorDirection;
        return this;
    }

    public void setMotorDirection(DcMotorSimple.Direction motorDirection) {
        launcher.setDirection(motorDirection);
    }

    public DcMotorSimple.Direction getMotorDirection() {
        return launcherDirection;
    }

    public void setLeftFeederDirection(DcMotorSimple.Direction motorDirection) {
        leftFeederDirection = motorDirection;
        leftFeeder.setDirection(motorDirection);
    }

    public void setRightFeederDirection(DcMotorSimple.Direction motorDirection) {
        rightFeederDirection = motorDirection;
        rightFeeder.setDirection(motorDirection);
    }

    public DcMotorSimple.Direction getLeftFeederDirection() {
        return leftFeederDirection;
    }

    public DcMotorSimple.Direction getRightFeederDirection() {
        return rightFeederDirection;
    }

    public boolean isBusy() {
        return isBusy;
    }

    public void setLauncherOff() {
        launch(false, false, false);
        launcher.setVelocity(0.0);
    }

    public void setLauncherPanic() {
        if (!isBusy()) {
            launch(false, false, true);
            while (isBusy())
                launch(false, false, false);
        }
    }

    public void launchCloseShot() {
        while (!isBusy())
            launch(true, false, false);
        while (isBusy())
            launch(false, false, false);
    }

    public void launchCloseShot(int count) {
        for (int i = 0; i < count; i++)
            launchCloseShot();
    }

    public void launchFarShot() {
        while (!isBusy())
            launch(false, true, false);
        while (isBusy())
            launch(false, false, false);
    }

    public void launchFarShot(int count) {
        for (int i = 0; i < count; i++)
            launchFarShot();
    }

    public void launch(boolean closeShotRequested, boolean farShotRequested, boolean panicRequested) {
        switch (launchState) {
            case IDLE:
                if (closeShotRequested) {
                    isBusy = true;
                    launchState = LaunchState.SPIN_UP_C;
                } else if (farShotRequested) {
                    isBusy = true;
                    launchState = LaunchState.SPIN_UP_F;
                } else if (panicRequested) {
                    isBusy = true;
                    setLeftFeederDirection(leftFeederDirection.inverted());
                    setRightFeederDirection(rightFeederDirection.inverted());
                    leftFeeder.setPower(1.0);
                    rightFeeder.setPower(1.0);
                    feederTimer.reset();
                    launchState = LaunchState.PANIC;
                } else {
                    isBusy = false;
                    launcher.setVelocity(0.0);
                }
                break;
            case PANIC:
                if (feederTimer.seconds() > feederRunSec) {
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    setLeftFeederDirection(leftFeederDirection.inverted());
                    setRightFeederDirection(rightFeederDirection.inverted());
                    launchState = LaunchState.IDLE;
                }
                break;
            case SPIN_UP_C:
                launcher.setVelocity(closeTargetVelocity);
                if (launcher.getVelocity() > closeMinVelocity)
                    launchState = LaunchState.LAUNCH;
                break;
            case SPIN_UP_F:
                launcher.setVelocity(farTargetVelocity);
                if (launcher.getVelocity() > farMinVelocity)
                    launchState = LaunchState.LAUNCH;
                break;
            case LAUNCH:
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(1.0);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > feederRunSec) {
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    launchTimer.reset();
                    launchState = LaunchState.COOL_OFF;
                }
                break;
            case COOL_OFF:
                if (launchTimer.seconds() > launcherCoolOffSec)
                    launchState = LaunchState.IDLE;
                break;
        }
    }

    public String getState() {
        return launchState.toString();
    }

    public double getLauncherVelocity() {
        return launcher.getVelocity();
    }

    public void setLauncherCloseVelocity(double target, double minimum) {
        closeTargetVelocity = target;
        closeMinVelocity = minimum;
    }

    public void setLauncherFarVelocity(double target, double minimum) {
        farTargetVelocity = target;
        farMinVelocity = minimum;
    }

    public void setFeederRunSec(double second) {
        feederRunSec = second;
    }

    public void setLauncherCoolOffSec(double second) {
        launcherCoolOffSec = second;
    }
}
