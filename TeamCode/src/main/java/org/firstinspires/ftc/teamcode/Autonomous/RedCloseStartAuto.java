package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCloseStartAuto", group = "Error404")
public class RedCloseStartAuto extends PedroPathingAutoOpMode {
    private final PedroPathingAutoOpMode myOpMode = new PedroPathingAutoOpMode();

    @Override
    public final void init() {
        myOpMode.gamepad1 = gamepad1;
        myOpMode.gamepad2 = gamepad2;
        myOpMode.telemetry = telemetry;
        myOpMode.hardwareMap = hardwareMap;
        myOpMode.useRedPose = true;
        myOpMode.useFarStartPose = false;
        myOpMode.useFarStopPose  = false;
        myOpMode.init();
    }
    @Override
    public final void init_loop() { myOpMode.init_loop(); }
    @Override
    public final void start() { myOpMode.start(); }
    @Override
    public final void loop() { myOpMode.loop(); }
    @Override
    public final void stop() { myOpMode.stop(); }
}