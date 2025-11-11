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

public class Constants {
   /* public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.428)
            .forwardZeroPowerAcceleration(-44.347800)
            .lateralZeroPowerAcceleration(-86.786322)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.075,0.0,0.00001,0.6,0.01))
            .centripetalScaling(0.0008);
            */

   public static FollowerConstants followerConstants = new FollowerConstants()
           .mass(13.063)
           .forwardZeroPowerAcceleration(-46.194)
           .lateralZeroPowerAcceleration(-84.602)
           .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0, 0))
           .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.01))
           .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.075,0.0,0.00001,0.6,0.01))
           .centripetalScaling(0.0008);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /*
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
            .xVelocity(63.2593134)
            .yVelocity(47.1911640);

     */

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
            .xVelocity(57.511)
            .yVelocity(40.745);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(+6.75)
            .strafePodX(-2.5)
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
