package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
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
            .mass(10.4)
            .forwardZeroPowerAcceleration(-32.4627)
            .lateralZeroPowerAcceleration(-56.493)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0, 0.01, 0.01))
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.01,0, 0.001, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0.03,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1, 0, 0.1, 0.06, 0))
//            .centripetalScaling(0.0011);
            .centripetalScaling(0.0);

    public static MecanumConstants mecanumConstants =  new MecanumConstants()
            .maxPower(1.0)
            .xVelocity(65.7391)
            .yVelocity(56.276)
            .leftFrontMotorName("Wheel1")
            .rightFrontMotorName("Wheel2")
            .leftRearMotorName("Wheel3")
            .rightRearMotorName("Wheel4")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.MM)
            .strafePodX(-24 * 7)
            .forwardPodY(-56)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.97,
            100,
            2,
            2);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
