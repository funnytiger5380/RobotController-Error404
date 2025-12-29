package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static double ROBOT_LENGTH_INCHES = 17.75;
    public static double ROBOT_WIDTH_INCHES = 16.00;
    public static double WHEEL_CIRCUMFERENCE_MM = 104;
    public static double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_CIRCUMFERENCE_MM / 25.4;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-25.39047) // from tuning
            .lateralZeroPowerAcceleration(-48.92773) // from tuning
            .translationalPIDFCoefficients(new PIDFCoefficients(0.09,0,0.01,0.03)) // from tuning
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0,0.02)) // from tuning
            .automaticHoldEnd(true)
            .mass(11.0); // weight in kilogram

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName("left_drive_front")
            .leftRearMotorName("left_drive_back")
            .rightFrontMotorName("right_drive_front")
            .rightRearMotorName("right_drive_back")
            .xVelocity(65.51166) // from tuning
            .yVelocity(50.61879) // from tuning
            .maxPower(1.0)
            .nominalVoltage(12.0)
            .useVoltageCompensation(true)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.0) // forward X-pod Y offset (left positive) from robot centric
            .strafePodX(-1.875) // strafe Y-pod X offset (forward positive) from robot centric
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED) // X-pod on pinpoint device
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED); // Y-pod on pinpoint device

    public static PathConstraints pathConstraints = new PathConstraints(0.995,100,
            1.35,1.5); // from tuning

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}