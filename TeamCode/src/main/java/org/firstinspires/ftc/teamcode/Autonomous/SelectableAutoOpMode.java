package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "SelectableAutoOpMode", group = "Error404")
public class SelectableAutoOpMode extends SelectableOpMode {

    public SelectableAutoOpMode() {
        super("<< Select a Autonomous OpMode >>", m -> {
            m.folder("Blue Alliance", r -> {
                r.add("Blue Close Start Pose", BlueCloseStartAuto::new);
                r.add("Blue Far Start Pose", BlueFarStartAuto::new);
            });
            m.folder("Red Alliance", f -> {
                f.add("Red Close Start Pose", RedCloseStartAuto::new);
                f.add("Red Far Start Pose", RedFarStartAuto::new);
            });
        });
    }
}
