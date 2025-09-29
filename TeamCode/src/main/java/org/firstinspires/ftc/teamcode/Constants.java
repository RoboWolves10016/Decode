package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.443)
            .forwardZeroPowerAcceleration(-34.941349752350256)
            .lateralZeroPowerAcceleration(-52.014785770225394)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2,0, 0.005, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(2,0,0.1,0.02));

    public static MecanumConstants mecanumConstants =  new MecanumConstants()
            .maxPower(1)
            .xVelocity(61.43478129229207)
            .yVelocity(53.57349498777707)
            .leftFrontMotorName("Wheel1")
            .rightFrontMotorName("Wheel2")
            .leftRearMotorName("Wheel3")
            .rightRearMotorName("Wheel4")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.METER)
            .strafePodX(0.024 * 7)
            .forwardPodY(0.024 * 2.5)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

//    public static DriveEncoderConstants encoderConstants = new DriveEncoderConstants()
//            .robotWidth(365.7) //mm
//            .robotLength(0.024 * 14) //mm
//            .forwardTicksToInches(3805.67)
//            .strafeTicksToInches(31048.49)
//            .turnTicksToInches(6.344745)
//            .leftFrontMotorName("Wheel0")
//            .rightFrontMotorName("Wheel1")
//            .leftRearMotorName("Wheel2")
//            .rightRearMotorName("Wheel3")
//            .leftFrontEncoderDirection(Encoder.REVERSE)
//            .rightFrontEncoderDirection(Encoder.FORWARD)
//            .leftRearEncoderDirection(Encoder.REVERSE)
//            .rightRearEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
