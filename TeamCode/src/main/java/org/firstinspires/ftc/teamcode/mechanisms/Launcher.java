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
    private String leftFeederName = "leftFeeder";
    private String rightFeederName = "rightFeeder";

    DcMotorSimple.Direction leftFeederDirection = FORWARD;
    DcMotorSimple.Direction rightFeederDirection = REVERSE;

    private DcMotorEx launcher;
    private String launcherName = "launcher";
    DcMotorSimple.Direction launcherDirection = FORWARD;
    DcMotor.ZeroPowerBehavior laucherZeroPowerBehavior = BRAKE;

    private double readyTargetVelocity = 1200;
    private double closeTargetVelocity = 1300;
    private double closeMinVelocity    = 1280;
    private double farTargetVelocity   = 1600;
    private double farMinVelocity      = 1580;

    private double feederRunSec = 0.15;
    private double panicRunSec = 0.10;
    private double launcherCoolOffSec = 0.20;
    private double launcherOnSecAtIdle = 2.0;

    private enum LaunchState { IDLE, PANIC, SPIN_UP_F, SPIN_UP_C, LAUNCH, LAUNCHING, RETRACT, COOL_OFF }
    private LaunchState launchState = LaunchState.IDLE;
    private boolean launcherOnAtIdle = false;
    private volatile boolean isBusy = false;
    private volatile boolean isPanic = false;

    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime launchTimer = new ElapsedTime();

    public void build(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, launcherName);
        launcher.setMode(RUN_USING_ENCODER);
        launcher.setDirection(launcherDirection);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher.setVelocity(0.0);
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
        leftFeeder.setDirection(leftFeederDirection);
    }

    public void setRightFeederDirection(DcMotorSimple.Direction motorDirection) {
        rightFeederDirection = motorDirection;
        rightFeeder.setDirection(rightFeederDirection);
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

    public boolean isPanic() {
        return isPanic;
    }

    public void launcherOnAtIdle() {
        launcherOnAtIdle = true;
    }

    public void launcherOffAtIdle() {
        launcherOnAtIdle = false;
    }

    public void setLauncherOff() {
        launcher.setVelocity(0.0);
        leftFeeder.setPower(0.0);
        rightFeeder.setPower(0.0);
        leftFeeder.setDirection(leftFeederDirection);
        rightFeeder.setDirection(rightFeederDirection);
        launchState = LaunchState.IDLE;
    }

    public void launchCloseShot() {
        launch(true, false, false, false);
        // wait until the current shot is done
        do { launch(false, false, false, false); } while (isBusy);
    }

    public void launchCloseShot(int count, double interval) {
        ElapsedTime intervalTimer = new ElapsedTime();

        launcherOnAtIdle(); // set to keep launcher fly wheel on between multiple launches
        for (int i = count; i > 0; i--) {
            if (i == 1) {
                launcherOffAtIdle();
                launchCloseShot();
            } else {
                launchCloseShot();
                intervalTimer.reset();
                do {} while (intervalTimer.seconds() < interval); // interval between launches
            }
        }
    }

    public void launchFarShot() {
        launch(false, true, false, false);
        // wait until the current shot is done
        do { launch(false, false, false, false); } while (isBusy);
    }

    public void launchFarShot(int count, double interval) {
        ElapsedTime intervalTimer = new ElapsedTime();

        launcherOnAtIdle(); // set to keep launcher fly wheel on between multiple launches
        for (int i = count; i > 0; i--) {
            if (i == 1) {
                launcherOffAtIdle();
                launchFarShot();
            } else {
                launchFarShot();
                intervalTimer.reset();
                do {} while (intervalTimer.seconds() < interval); // interval between launches
            }
        }
    }

    public void launch(boolean closeShot, boolean farShot) {
        launch(closeShot, farShot, false, false);
    }

    public void launch(boolean closeShot, boolean farShot, boolean getReady, boolean panic) {
        switch (launchState) {
            case IDLE:
                if (farShot) {
                    isBusy = true;
                    launchState = LaunchState.SPIN_UP_F;
                } else if (closeShot) {
                    isBusy = true;
                    launchState = LaunchState.SPIN_UP_C;
                } else if (panic) {
                    isBusy = true;
                    isPanic = true;
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    feederTimer.reset();
                    launchState = LaunchState.PANIC;
                } else {
                    isBusy = false;
                }

                if (launcherOnAtIdle) {
                    if (getReady) {
                        launchTimer.reset();
                        launcher.setVelocity(readyTargetVelocity);
                    } else if (launchTimer.seconds() > launcherOnSecAtIdle)
                        launcher.setVelocity(0.0);
                } else {
                    launcher.setVelocity(0.0);
                }
                break;

            case PANIC:
                if (farShot) {
                    isPanic = false;
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    launchState = LaunchState.SPIN_UP_F;
                }
                else if (closeShot) {
                    isPanic = false;
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    launchState = LaunchState.SPIN_UP_C;
                }
                else {
                    if (feederTimer.seconds() > panicRunSec) {
                        isPanic = false;
                        leftFeeder.setPower(0.0);
                        rightFeeder.setPower(0.0);
                        launchState = LaunchState.IDLE;
                    }
                    if (launcherOnAtIdle) {
                        if (getReady) {
                            launchTimer.reset();
                            launcher.setVelocity(readyTargetVelocity);
                        }
                    }
                }
                break;

            case SPIN_UP_F:
                launcher.setVelocity(farTargetVelocity);
                if (launcher.getVelocity() > farMinVelocity)
                    launchState = LaunchState.LAUNCH;
                break;

                case SPIN_UP_C:
                launcher.setVelocity(closeTargetVelocity);
                if (launcher.getVelocity() > closeMinVelocity)
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
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    feederTimer.reset();
                    launchState = LaunchState.COOL_OFF;
                }
                break;

            case COOL_OFF:
                if (feederTimer.seconds() > launcherCoolOffSec) {
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    launchTimer.reset();
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public String getState() {
        return launchState.toString();
    }

    public double getLauncherVelocity() {
        return launcher.getVelocity();
    }

    public void setLauncherReadyVelocity(double target) {
        readyTargetVelocity = target;
    }

    public void setLauncherCloseVelocity(double target, double minimum) {
        closeTargetVelocity = target;
        closeMinVelocity = minimum;
    }

    public void setLauncherFarVelocity(double target, double minimum) {
        farTargetVelocity = target;
        farMinVelocity = minimum;
    }

    public void setLauncherCoolOffSec(double second) {
        launcherCoolOffSec = second;
    }

    public void setLauncherOnSecAtIdle(double second) {
        launcherOnSecAtIdle = second;
    }

    public void setFeederRunSec(double second) {
        feederRunSec = second;
    }

    public void setPanicRunSec(double second) {
        panicRunSec = second;
    }
}