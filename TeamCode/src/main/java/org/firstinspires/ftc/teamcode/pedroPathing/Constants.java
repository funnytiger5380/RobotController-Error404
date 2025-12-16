package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static double ROBOT_WIDTH_INCHES = 15.75;
    public static double ROBOT_LENGTH_INCHES = 17;
    public static double WHEEL_CIRCUMFERENCE_MM = 104;
    public static double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_CIRCUMFERENCE_MM / 25.4;

    public static double FORWARD_TICKS_TO_INCHES = 0.010; // from tuning
    public static double STRAFE_TICKS_TO_INCHES = 0.020;  // from tuning
    public static double TURN_TICKS_TO_INCHES = 0.013;    // from tuning

    public static FollowerConstants followerConstants = new FollowerConstants()
            .automaticHoldEnd(true)
            .mass(10.9); // weight in kg

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName("left_drive_front")
            .leftRearMotorName("left_drive_back")
            .rightFrontMotorName("right_drive_front")
            .rightRearMotorName("right_drive_back")
            .maxPower(1.0)
            .nominalVoltage(12.0)
            .useVoltageCompensation(true)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.0) // forward pod offset (left positive) from center of robot
            .strafePodX(1.875) // strafe pod offset (forward positive) from center of robot
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}