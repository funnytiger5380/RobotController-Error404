package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "BlueRobotCentricTeleOp", group = "Error404")
public class BlueRobotCentricTeleOp extends PedroPathingTeleOp {
    private final PedroPathingTeleOp myOpMode = new PedroPathingTeleOp();

    @Override
    public void init() {
        myOpMode.gamepad1 = gamepad1;
        myOpMode.gamepad2 = gamepad2;
        myOpMode.telemetry = telemetry;
        myOpMode.hardwareMap = hardwareMap;
        myOpMode.isRobotCentric = true;
        myOpMode.isIMUResetRequested = false;
        myOpMode.alliance = Alliance.BLUE;
        myOpMode.init();
    }

    @Override
    public void start() { myOpMode.start(); }

    @Override
    public void loop() { myOpMode.loop(); }
}