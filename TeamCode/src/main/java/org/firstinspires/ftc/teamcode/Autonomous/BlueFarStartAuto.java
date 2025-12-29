package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueFarStartAuto", group = "Error404")
public class BlueFarStartAuto extends PedroPathingOpMode {
    PedroPathingOpMode myOpMode = new PedroPathingOpMode();

    @Override
    public final void runOpMode() {
        myOpMode.gamepad1 = gamepad1;
        myOpMode.gamepad2 = gamepad2;
        myOpMode.telemetry = telemetry;
        myOpMode.hardwareMap = hardwareMap;
        myOpMode.useRedPose(false);
        myOpMode.useFarStartPose(true);
        myOpMode.useFarStopPose(true);
        myOpMode.runOpMode();
    }
}
