package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("lfd")
            .leftRearMotorName("lbd")
            .rightFrontMotorName("rfd")
            .rightRearMotorName("rbd")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(78.53294793076402)
            .yVelocity(66.05725530188853);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.1)
            .forwardZeroPowerAcceleration(-40.81561056531705)
            .lateralZeroPowerAcceleration(-59.15365907069915)
//            .translationalPIDFCoefficients(new PIDFCoefficients(
//                    0.14,
//                    0,
//                    0.02,
//                    0.015
//            ))
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.04,
                    0,
                    0,
                    0.015
            ))
            .translationalPIDFSwitch(2)
////            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
////                    0.06,
////                    0,
////                    0.0035,                     0.004,
////                    0.015                       0.02
////            ))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.4,
                    0,
                    0.005,
                    0.0006
            ))
            .useSecondaryTranslationalPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.64,
                    0,
                    0.022,
                    0.02
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.1,
                    0.0005
            ))
            .useSecondaryHeadingPIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.0035,
                    0,
                    0.00032,
                    0.6,
                    0.045
            ))
            .useSecondaryDrivePIDF(true)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))

            .drivePIDFSwitch(15)
            .headingPIDFSwitch(0.07)
            .holdPointHeadingScaling(35)
            .holdPointTranslationalScaling(0.25)
            .turnHeadingErrorThreshold(0.01)
            .automaticHoldEnd(true)
            .centripetalScaling(0.0005);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.52)
            .strafePodX(-3.38583)
            .hardwareMapName("pinpoint")

            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            100,
            1.6,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}