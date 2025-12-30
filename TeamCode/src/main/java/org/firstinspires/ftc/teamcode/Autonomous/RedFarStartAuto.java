package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedFarStartAuto", group = "Error404")
public class RedFarStartAuto extends PedroPathingOpMode {
    private final PedroPathingOpMode myOpMode = new PedroPathingOpMode();

    @Override
    public final void runOpMode() {
        myOpMode.gamepad1 = gamepad1;
        myOpMode.gamepad2 = gamepad2;
        myOpMode.telemetry = telemetry;
        myOpMode.hardwareMap = hardwareMap;
        myOpMode.useRedPose = true;
        myOpMode.useFarStartPose = true;
        myOpMode.useFarStopPose  = true;
        myOpMode.runOpMode();
    }
}
