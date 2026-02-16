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
    DcMotorSimple.Direction leftFeederDirectionInv = REVERSE;

    DcMotorSimple.Direction rightFeederDirection = REVERSE;
    DcMotorSimple.Direction rightFeederDirectionInv = FORWARD;

    private DcMotorEx launcher;
    private String launcherName = "launcher";
    DcMotorSimple.Direction launcherDirection = FORWARD;
    DcMotor.ZeroPowerBehavior laucherZeroPowerBehavior = BRAKE;

    private double closeTargetVelocity = 1300;
    private double closeMinVelocity    = 1275;
    private double farTargetVelocity   = 1650;
    private double farMinVelocity      = 1640;

    private double feederRunSec = 0.15;
    // UPDATED: Set to 2.1 seconds as requested
    private double launcherCoolOffSec = 2.1;

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
        launcher.setZeroPowerBehavior(laucherZeroPowerBehavior);
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
        leftFeederDirectionInv = leftFeederDirection.inverted();
        return this;
    }

    public Launcher rightFeederDirection(DcMotorSimple.Direction motorDirection) {
        rightFeederDirection = motorDirection;
        rightFeederDirectionInv = rightFeederDirection.inverted();
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
        leftFeederDirectionInv = leftFeederDirection.inverted();
        leftFeeder.setDirection(leftFeederDirection);
    }

    public void setRightFeederDirection(DcMotorSimple.Direction motorDirection) {
        rightFeederDirection = motorDirection;
        rightFeederDirectionInv = rightFeederDirection.inverted();
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
        launch(true, false, false);
        do { launch(false, false, false); } while (isBusy);
    }

    public void launchCloseShot(int count) {
        launchCloseShot(count, 0.0);
    }

    public void launchCloseShot(int count, double interval) {
        launcherOnAtIdle();
        for (int i = count; i > 0; i--) {
            if (i == 1) {
                launcherOffAtIdle();
                launchCloseShot();
            } else {
                launchCloseShot();
                launchTimer.reset();
                do {} while (launchTimer.seconds() < interval);
            }
        }
    }

    public void launchFarShot() {
        launch(false, true, false);
        do { launch(false, false, false); } while (isBusy);
    }

    public void launchFarShot(int count) {
        launchFarShot(count, 0.0);
    }

    public void launchFarShot(int count, double interval) {
        launcherOnAtIdle();
        for (int i = count; i > 0; i--) {
            if (i == 1) {
                launcherOffAtIdle();
                launchFarShot();
            } else {
                launchFarShot();
                launchTimer.reset();
                do {} while (launchTimer.seconds() < interval);
            }
        }
    }

    public void launch(boolean closeShot, boolean farShot) {
        launch(closeShot, farShot, false);
    }

    public void launch(boolean closeShot, boolean farShot, boolean panic) {
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
                    leftFeeder.setDirection(leftFeederDirectionInv);
                    rightFeeder.setDirection(rightFeederDirectionInv);
                    leftFeeder.setPower(1.0);
                    rightFeeder.setPower(1.0);
                    feederTimer.reset();
                    launchState = LaunchState.PANIC;
                } else {
                    isBusy = false;
                    if (!launcherOnAtIdle)
                        launcher.setVelocity(0.0);
                }
                break;
            case PANIC:
                if (farShot) {
                    isPanic = false;
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    leftFeeder.setDirection(leftFeederDirection);
                    rightFeeder.setDirection(rightFeederDirection);
                    launchState = LaunchState.SPIN_UP_F;
                }
                else if (closeShot) {
                    isPanic = false;
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    leftFeeder.setDirection(leftFeederDirection);
                    rightFeeder.setDirection(rightFeederDirection);
                    launchState = LaunchState.SPIN_UP_C;
                }
                else if (feederTimer.seconds() > feederRunSec) {
                    isPanic = false;
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    leftFeeder.setDirection(leftFeederDirection);
                    rightFeeder.setDirection(rightFeederDirection);
                    launchState = LaunchState.IDLE;
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
                if (feederTimer.seconds() > 0.15) {
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    feederTimer.reset();
                    launchState = LaunchState.RETRACT;
                }
                break;
            case RETRACT:
                if (feederTimer.seconds() > 0.3) {
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    launchTimer.reset();
                    launchState = LaunchState.COOL_OFF;
                }
                break;
            case COOL_OFF:
                // Check if trigger is pulled again to reset the cycle
                if (farShot) {
                    isBusy = true;
                    launchState = LaunchState.SPIN_UP_F;
                } else if (closeShot) {
                    isBusy = true;
                    launchState = LaunchState.SPIN_UP_C;
                }
                // Wait for the 2.1s timer to expire
                else if (launchTimer.seconds() > launcherCoolOffSec) {
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