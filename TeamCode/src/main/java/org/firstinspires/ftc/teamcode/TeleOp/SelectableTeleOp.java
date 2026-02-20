package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is the SelectableTeleOp class. It contains a selection menu for various PedroPathingTeleOp
 * OpModes.
 */
@Disabled
@TeleOp(name = "SelectableTeleOp", group = "Error404")
public class SelectableTeleOp extends SelectableOpMode {

    public SelectableTeleOp() {
        super("<< Select a TeleOp mode >>", m -> {
            m.folder("Robot Centric", r -> {
                r.add("Blue Robot Centric", BlueRobotCentricTeleOp::new);
                r.add("Red Robot Centric", RedRobotCentricTeleOp::new);
            });
            m.folder("Field Centric", f -> {
                f.add("Blue Field Centric", BlueFieldCentricTeleOp::new);
                f.add("Red Field Centric", RedFieldCentricTeleOp::new);
                f.add("Practice: Blue Field Centric", PracticeBlueFieldCentric::new);
                f.add("Practice: Red Field Centric", PracticeRedFieldCentric::new);
            });
        });
    }
}

/**
 * This is the Blue alliance Robot centric TeleOp OpMode.
 * The robot runs with the robot centric control. The robot drives facing forward when the control
 * pushes forward disregards to the current position it stands in the field.
 */
class BlueRobotCentricTeleOp extends PedroPathingTeleOp {
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

/**
 * This is the Red alliance Robot centric TeleOp OpMode.
 * The robot runs with the robot centric control. The robot drives facing forward when the control
 * pushes forward disregards to the current position it stands in the field.
 */
class RedRobotCentricTeleOp extends PedroPathingTeleOp {
    private final PedroPathingTeleOp myOpMode = new PedroPathingTeleOp();

    @Override
    public void init() {
        myOpMode.gamepad1 = gamepad1;
        myOpMode.gamepad2 = gamepad2;
        myOpMode.telemetry = telemetry;
        myOpMode.hardwareMap = hardwareMap;
        myOpMode.isRobotCentric = true;
        myOpMode.isIMUResetRequested = false;
        myOpMode.alliance = Alliance.RED;
        myOpMode.init();
    }
    @Override
    public void start() { myOpMode.start(); }
    @Override
    public void loop() { myOpMode.loop(); }
}

/**
 * This is the Blue alliance Field centric TeleOp OpMode.
 * The robot runs with the field centric control, which drives in the direction of a heading with
 * respect to the Pedro-pathing field coordinate system. As Blue alliance, the robot runs
 * perpendicular to Blue gate wall (180 degree) when the control pushes forward. The robot uses the
 * Autonomous end position to estimate its forward heading for the control when the field
 * centric TeleOp is initialized.
 */
class BlueFieldCentricTeleOp extends PedroPathingTeleOp {
    private final PedroPathingTeleOp myOpMode = new PedroPathingTeleOp();

    @Override
    public void init() {
        myOpMode.gamepad1 = gamepad1;
        myOpMode.gamepad2 = gamepad2;
        myOpMode.telemetry = telemetry;
        myOpMode.hardwareMap = hardwareMap;
        myOpMode.isRobotCentric = false;
        myOpMode.isIMUResetRequested = false;
        myOpMode.alliance = Alliance.BLUE;
        myOpMode.init();
    }
    @Override
    public void start() { myOpMode.start(); }
    @Override
    public void loop() { myOpMode.loop(); }
}

/**
 * This is the Red alliance Field centric TeleOp OpMode.
 * The robot runs with the field centric control, which drives in the direction of a heading with
 * respect to the Pedro-pathing field coordinate system. As Red alliance, the robot runs
 * perpendicular to Red gate wall (0 degree) when the control pushes forward. The robot uses the
 * Autonomous end position to estimate its forward heading for the control when the field
 * centric TeleOp is initialized.
 */
class RedFieldCentricTeleOp extends PedroPathingTeleOp {
    private final PedroPathingTeleOp myOpMode = new PedroPathingTeleOp();

    @Override
    public void init() {
        myOpMode.gamepad1 = gamepad1;
        myOpMode.gamepad2 = gamepad2;
        myOpMode.telemetry = telemetry;
        myOpMode.hardwareMap = hardwareMap;
        myOpMode.isRobotCentric = false;
        myOpMode.isIMUResetRequested = false;
        myOpMode.alliance = Alliance.RED;
        myOpMode.init();
    }
    @Override
    public void start() { myOpMode.start(); }
    @Override
    public void loop() { myOpMode.loop(); }
}

/**
 * This is the Blue alliance Field centric TeleOp OpMode for the practice.
 * The robot runs with the field centric control, which drives in the direction of a heading with
 * respect to the Pedro-pathing field coordinate system. As Blue alliance, the robot runs
 * perpendicular to Blue gate wall (180 degree) when the control pushes forward. For the practice,
 * the robot is set with its forward heading straightly away from the audience (90 degree) to begin
 * with when this TeleOp OpMode is initialized.
 */
class PracticeBlueFieldCentric extends PedroPathingTeleOp {
    private final PedroPathingTeleOp myOpMode = new PedroPathingTeleOp();

    @Override
    public void init() {
        myOpMode.gamepad1 = gamepad1;
        myOpMode.gamepad2 = gamepad2;
        myOpMode.telemetry = telemetry;
        myOpMode.hardwareMap = hardwareMap;
        myOpMode.isRobotCentric = false;
        myOpMode.isIMUResetRequested = true;
        myOpMode.alliance = Alliance.BLUE;
        myOpMode.init();
    }
    @Override
    public void start() { myOpMode.start(); }
    @Override
    public void loop() { myOpMode.loop(); }
}

/**
 * This is the Red alliance Field centric TeleOp OpMode for the practice.
 * The robot runs with the field centric control, which drives in the direction of a heading with
 * respect to the Pedro-pathing field coordinate system. As Red alliance, the robot runs
 * perpendicular to Red gate wall (0 degree) when the control pushes forward. For the practice,
 * the robot is set with its forward heading straightly away from the audience (90 degree) to begin
 * with when this TeleOp OpMode is initialized.
 */
class PracticeRedFieldCentric extends PedroPathingTeleOp {
    private final PedroPathingTeleOp myOpMode = new PedroPathingTeleOp();

    @Override
    public void init() {
        myOpMode.gamepad1 = gamepad1;
        myOpMode.gamepad2 = gamepad2;
        myOpMode.telemetry = telemetry;
        myOpMode.hardwareMap = hardwareMap;
        myOpMode.isRobotCentric = false;
        myOpMode.isIMUResetRequested = true;
        myOpMode.alliance = Alliance.RED;
        myOpMode.init();
    }
    @Override
    public void start() { myOpMode.start(); }
    @Override
    public void loop() { myOpMode.loop(); }
}