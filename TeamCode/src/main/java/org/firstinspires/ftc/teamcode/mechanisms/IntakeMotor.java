package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeMotor {
    private DcMotor intakeMotor;
    private String motorName = "intakeMotor";

    DcMotorSimple.Direction motorDirection = FORWARD;
    DcMotor.ZeroPowerBehavior motorZeroPowerBehavior = BRAKE;

    private enum IntakeDirection { FORWARD, REVERSE }
    private IntakeDirection intakeDirection = IntakeDirection.FORWARD;

    private enum IntakeState { OFF, ON, PANIC, MAX }
    private IntakeState intakeState = IntakeState.OFF;

    private double maxPower = 1.0;
    private double motorPower = 0.75;

    private final ElapsedTime timer = new ElapsedTime();
    private double panicTime = 0.10;
    private double maxTime = 0.20;

    private volatile boolean isBusy = false;
    private volatile boolean isPanic = false;

    public void build(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, motorName);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(motorDirection);
        intakeMotor.setZeroPowerBehavior(motorZeroPowerBehavior);
        intakeMotor.setPower(0.0);
    }

    public IntakeMotor motorName(String motorName) {
        this.motorName = motorName;
        return this;
    }

    public IntakeMotor motorDirection(DcMotorSimple.Direction motorDirection) {
        this.motorDirection = motorDirection;
        return this;
    }

    public IntakeMotor motorUseBrakeMode(boolean useBrakeMode) {
        if (useBrakeMode)
            motorZeroPowerBehavior = BRAKE;
        else
            motorZeroPowerBehavior = FLOAT;
        return this;
    }

    public IntakeMotor maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public IntakeMotor motorPower(double motorPower) {
        this.motorPower = motorPower;
        return this;
    }

    public String getMotorDirection() {
        return intakeDirection.toString();
    }

    public void setMotorDirectionForward() {
        motorDirection = FORWARD;
        intakeMotor.setDirection(motorDirection);
    }

    public void setMotorDirectionReverse() {
        motorDirection = REVERSE;
        intakeMotor.setDirection(motorDirection);
    }

    public void setMotorBrakeMode() {
        intakeMotor.setZeroPowerBehavior(BRAKE);
    }

    public void setMotorFloatMode() {
        intakeMotor.setZeroPowerBehavior(FLOAT);
    }

    public void setPower(double setPower) {
        motorPower = Math.min(setPower, maxPower);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public String getState() {
        return intakeState.toString();
    }

    public boolean isBusy() {
        return isBusy;
    }

    public boolean isPanic() {
        return isPanic;
    }

    public void setPanicTime(double panicTime) {
        this.panicTime = panicTime;
    }

    public void setMaxTime(double maxTime) {
        this.maxTime = maxTime;
    }
    public void setIntakeOff() {
        run(false, true, false);
    }

    public void setIntakeOn() {
        run(true, false, false);
    }

    public void setIntakeOn(double setPower) {
        setPower(setPower);
        setIntakeOn();
    }

    public void setIntakePanic() {
        run(false, false, true);
    }

    public void setIntakePanic(double panicTime) {
        double holdTime = this.panicTime;

        setPanicTime(panicTime);
        setIntakePanic();
        setPanicTime(holdTime);
    }

    public void setIntakePanic(double panicTime, double power) {
        double holdPower = this.motorPower;

        setPower(power);
        setIntakePanic(panicTime);
        setPower(holdPower);
    }

    public void run(boolean intakeOn, boolean intakeOff, boolean panic) {
        switch (intakeState) {
            case OFF:
                if (intakeOn) {
                    isBusy = true;
                    isPanic = false;
                    intakeState = IntakeState.ON;
                    intakeDirection = IntakeDirection.FORWARD;
                    intakeMotor.setPower(motorPower);
                } else if (panic) {
                    isBusy = true;
                    isPanic = true;
                    timer.reset();
                    intakeState = IntakeState.PANIC;
                    intakeDirection = IntakeDirection.REVERSE;
                    intakeMotor.setPower(-1 * motorPower);
                }
                break;

            case ON:
                if (intakeOff) {
                    isBusy = false;
                    intakeState = IntakeState.OFF;
                    intakeMotor.setPower(0.0);
                } else if (panic) {
                    isPanic = true;
                    timer.reset();
                    intakeState = IntakeState.PANIC;
                    intakeDirection = IntakeDirection.REVERSE;
                    intakeMotor.setPower(-1 * motorPower);
                }
                break;

            case PANIC:
                if (intakeOff) {
                    isBusy = false;
                    isPanic = false;
                    intakeState = IntakeState.OFF;
                    intakeMotor.setPower(0.0);
                } else if (timer.seconds() >= panicTime) {
                    isPanic = false;
                    timer.reset();
                    intakeState = IntakeState.MAX;
                    intakeDirection = IntakeDirection.FORWARD;
                    intakeMotor.setPower(maxPower);
                }
                break;

            case MAX:
                if (intakeOff) {
                    isBusy = false;
                    intakeState = IntakeState.OFF;
                    intakeMotor.setPower(0.0);
                } else if (timer.seconds() >= maxTime) {
                    intakeState = IntakeState.ON;
                    intakeMotor.setPower(motorPower);
                }
                break;
        }
    }
}
