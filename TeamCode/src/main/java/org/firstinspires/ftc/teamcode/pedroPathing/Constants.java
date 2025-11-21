package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Black Robot Constants - Camerupt
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.063)
            .forwardZeroPowerAcceleration(-43.275)
            .lateralZeroPowerAcceleration(-77.62)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.075,0.0,0.00001,0.6,0.01))
            .centripetalScaling(0.0009);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RFdrive")
            .rightRearMotorName("RBdrive")
            .leftRearMotorName("LBdrive")
            .leftFrontMotorName("LFdrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(59.19)
            .yVelocity(48.04);



    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(+6.88)
            .strafePodX(-2.40)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

//Green Robot Constants - Numel
/*
public class Constants {

   public static FollowerConstants followerConstants = new FollowerConstants()
           .mass(13.063)
           .forwardZeroPowerAcceleration(-45.60)
           .lateralZeroPowerAcceleration(-79.02)
           .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0, 0))
           .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.01))
           .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.075,0.0,0.00001,0.6,0.01))
           .centripetalScaling(0.0008);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RFdrive")
            .rightRearMotorName("RBdrive")
            .leftRearMotorName("LBdrive")
            .leftFrontMotorName("LFdrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(58.45)
            .yVelocity(45.12);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(+6.88)
            .strafePodX(-2.40)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

 */
