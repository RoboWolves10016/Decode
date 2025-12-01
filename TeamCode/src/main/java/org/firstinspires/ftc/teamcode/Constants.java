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
            .mass(10.93)
            .forwardZeroPowerAcceleration(-27.93978)
            .lateralZeroPowerAcceleration(-52.3840)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0, 0.01, 0.00))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0,0,0.05,0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.001, 0.06, 0))
            .centripetalScaling(0.0004);

    public static MecanumConstants mecanumConstants =  new MecanumConstants()
            .maxPower(1.0)
            .xVelocity(80.2044)
            .yVelocity(63.8607)
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
            .strafePodX(-24 * 6)
            .forwardPodY(-56)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.98,
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
