/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@Autonomous(name = "MechbotObjDetection", group = "Sensor")
//@Disabled
public class MechBotObjectDetection extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu_IMU;
    YawPitchRollAngles orientation;

    boolean isRobotMoving;

    private DcMotor RBdrive;
    private DcMotor RFdrive;
    private DcMotor LFdrive;
    private DcMotor LBdrive;
    int GC_dt_wheel_diameter;
    double GC_dt_gear_ratio;
    int GC_encoder_count;

    double CAMERA_HEIGHT_INCHES = 5.4; // Measured height to lens
    double SPHERE_RADIUS_INCHES = 2.5;  // Center of 5" artifact (5 / 2)

    // Proportional Gains (Tune these values for your robot!)
    // Higher values make response faster; too high causes oscillation/jitter.
    private static final double STEER_P = 0.035;   // Gain for turning based on tx
    private static final double DRIVE_P = 0.05;    // Gain for forward/backward based on distance

    // Desired distance to stop away from the ball (in inches or cm, depending on your LL setup)
    private static final double DESIRED_DISTANCE = 5.0;
    // Minimum power to overcome friction/deadband
    private static final double MIN_POWER = 0.05;


    @Override
    public void runOpMode() throws InterruptedException
    {

        GC_dt_wheel_diameter = 100;
        // this is a "3:1" , "4:1", and a "4:1"
        // which is actually 2.89 and 3.61, equaling 37.66
        GC_dt_gear_ratio = 0.5;
        GC_encoder_count = 288;




        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu_IMU = hardwareMap.get(IMU.class, "imu");

        RBdrive = hardwareMap.get(DcMotor.class, "RBdrive");
        RFdrive = hardwareMap.get(DcMotor.class, "RFdrive");
        LFdrive = hardwareMap.get(DcMotor.class, "LFdrive");
        LBdrive = hardwareMap.get(DcMotor.class, "LBdrive");

        RBdrive.setDirection(DcMotor.Direction.REVERSE);
        RFdrive.setDirection(DcMotor.Direction.REVERSE);
        LFdrive.setDirection(DcMotor.Direction.FORWARD);
        LBdrive.setDirection(DcMotor.Direction.FORWARD);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.setMsTransmissionInterval(11);

        // This sets how often we ask Limelight for data (100 times per second)
        limelight.setPollRateHz(100);
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        Init_IMU();
        limelight.start();

        isRobotMoving = false;


        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            orientation = imu_IMU.getRobotYawPitchRollAngles();
            double yaw = orientation.getYaw(AngleUnit.DEGREES);
            //limelight.updateRobotOrientation(yaw);
            //getAprilTagFromLimelight(1);

            //getLimelightMountAngleDegrees();
            //getDistanceOfSample();

            //setMotorPowerTest();
            //telemetry.update();
            followObject();
            //break;
        }

        limelight.stop();
    }
/*
Direction Check: If the robot turns away from the ball, flip the sign of value assigned
to STEER_P. If it drives backward when it should go forward, flip the value assigned to
DRIVE_P
 */
private void followObject(){
    while (opModeIsActive()){
        LLResult result = limelight.getLatestResult();

        double drivePower = 0.0;
        double turnPower = 0.0;

        if (result != null && result.isValid()){
            // tx: Horizontal offset from crosshair to target (-31 to 31 degrees)
            double tx = result.getTx();

            double currentDistance = getDistanceOfSample();

            // Calculate Errors
            double turnError = tx;
            double distanceError = currentDistance - DESIRED_DISTANCE;

            if(distanceError <= 0){
                continue;
            }

            // Simple P-Controller calculation
            turnPower = turnError * STEER_P;

            // Drive forward if too far away; drive back if too close
            // Only adjust drive power if we're roughly pointing toward the target
            if (Math.abs(tx) < 15.0) {
                drivePower = distanceError * DRIVE_P;
            }

            // Apply power clamps (-0.7 to 0.7 for safety during testing)
           turnPower  = Math.max(-0.7, Math.min(0.7, turnPower));
           drivePower = Math.max(-0.6, Math.min(0.6, drivePower));

            //turnPower  = Math.max(-0.3, Math.min(0.3, turnPower));
           // drivePower = Math.max(-0.2, Math.min(0.2, drivePower));

            telemetry.addData("Target", "DETECTED");
            telemetry.addData("TX Offset", "%.2f deg", tx);
            telemetry.addData("Current Distance", "%.2f in", currentDistance);
        }else {
            telemetry.addData("Target", "NOT FOUND");
        }

        // Apply calculated powers to drive motors
        setMotorPowers(drivePower, turnPower);

        telemetry.addData("Drive Power", "%.2f", drivePower);
        telemetry.addData("Turn Power", "%.2f", turnPower);
        telemetry.update();

    }
}

    private void setMotorPowerTest() {

    if (isRobotMoving == false){
        telemetry.addData("INSIDE FUNCTION", "");
        double leftPower = 0.6;
        double rightPower = 0.6;
        LFdrive.setPower(leftPower);
       // LBdrive.setPower(leftPower);
       // RFdrive.setPower(rightPower);
       // RBdrive.setPower(rightPower);

        isRobotMoving = true;
    }

    }

    /**
     * Set arcade-style drive values to four drive motors
     */
    private void setMotorPowers(double drive, double turn) {
        double leftPower  = drive + turn;
        double rightPower = drive - turn;

        // Normalize motor powers if any exceeds 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        LFdrive.setPower(leftPower);
        LBdrive.setPower(leftPower);
        RFdrive.setPower(rightPower);
        RBdrive.setPower(rightPower);
    }

    /*
    For limelight mounted in an incline facing to the ground
    d = (h1-h2)/tan(a1+a2)  // originally suggested  d = (h1-h2)/tan(a1-a2)

    h1 = height of the limelight lens from the ground in inches
    h2 : 2.5 in our case as we have a sphere of 5 inches diameter
    a1: Calculated downward camera mount angles ( in degrees)
    a2: ty value reported by Limelight
     */
    private double getDistanceOfSample(){

        double distanceInches = 0.0;
        limelight.pipelineSwitch(0); //pipeline 0 is Object Detector

        while(opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
/*
        int counter = 0;
        while(detectorResults.size() <= 0) {
            result = limelight.getLatestResult();
            detectorResults = result.getDetectorResults();
            counter++;

            if(counter > 100){
                break;
            }
        }

 */

            if (detectorResults.size() > 0) {
                for (LLResultTypes.DetectorResult dr : detectorResults) {

                    //change this based on calculation from getLimelightMountAngleDegrees

                    double mountAngleDegrees = 11.445;

                    double ty = dr.getTargetYDegrees(); //a2 in deg
                    telemetry.addData("Detector", "Class: %s, TY: %.4f", dr.getClassName(), ty);

                    if(dr.getClassName().equals("green") ){
                        // Total angle from horizontal sightline down to the target
                        double totalAngleDegrees = mountAngleDegrees - ty;

                        // Convert angle to radians for Java's Math library
                        double totalAngleRadians = Math.toRadians(totalAngleDegrees);

                        // Height difference between camera lens and target center
                        double deltaH = CAMERA_HEIGHT_INCHES - SPHERE_RADIUS_INCHES;

                        // Calculate horizontal floor distance (d)
                        distanceInches = deltaH / Math.tan(totalAngleRadians);

                        telemetry.addData("Distance to Sphere", "%.2f inches", distanceInches);

                    }

                    //telemetry.update();
                    return distanceInches;


                }

            } else {
                telemetry.addData("Object not Detected", 1);
                //telemetry.update();
                return 0.0;
            }
        }
        return distanceInches;
    }

    /*
    assuming limelight is mounted facing towards the floor

    Object to be detected is a Sphere of 5 inch diameter ( artifact from Decode game)
    a1: Mount Angle ( we need to find this)
    a2: (ty from limelight) is positive if the sphere is above the center cross hair
        and negative if it is below the center cross hair

    h1: height of the center of limelight lens from the floor ( straight up height)
    2.5 : is the center of the sphere ( artifact) from the ground


    Total downward angle to the sphere center is ( a1 - a2)

    Solving for the downward mounting angle a1 = (arctan (h1 - 2.5)/d) - a2
    // originally suggested angle a1 = (arctan (h1 - 2.5)/d) + a2
    for a2 -> retain the value from ty as it is whether positive or negative
     */
    private void getLimelightMountAngleDegrees(){
        limelight.pipelineSwitch(0); //pipeline 2 is Object Detector

        while (true) {
            LLResult result = limelight.getLatestResult();

            // Access detector results
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            if( detectorResults.size() > 0) {
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    //telemetry.addData("Detector", "Class: %s, Area: %.4f", dr.getClassName(), dr.getTargetArea());


                    //double targetOffSetAngle_VerticalRaD = targetOffsetAngle_Vertical * ( 3.14159/180);
                    //double targetOffSetAngle_VerticalRaD = Math.toRadians(a2);

                    double cameraHeightInches = 5.4; // Measured height to lens
                    double targetHeightInches = 2.5;  // Center of 5" artifact (5 / 2)
                    double distanceInches = 21.5;     // Measured horizontal distance to sphere

                    // 2. Read 'ty' from Limelight NetworkTable
                    double tyDegrees = dr.getTargetYDegrees(); //a2
                    telemetry.addData("Detector", "Class: %s, TY: %.4f", dr.getClassName(), tyDegrees);

                    // 3. Calculate delta height
                    double deltaY = cameraHeightInches - targetHeightInches;
                    double deltaX = distanceInches;

                    // 4. Calculate camera pitch angle using Math.atan2
                    double angleFromHorizontalRad = Math.atan2(deltaY, deltaX);
                    double angleFromHorizontalDeg = Math.toDegrees(angleFromHorizontalRad) + tyDegrees;


                    telemetry.addData("LimeLight MountAngle in DEG ", "%.6f", angleFromHorizontalDeg);

                    telemetry.update();
                    sleep(20000);
                    break;


                }
            } /*else {
                telemetry.addData ("Object not Detected", 1);
                telemetry.update();
            } */

            if( detectorResults.size() > 0)
                break;

        }

    }



    private void getAprilTagFromLimelight(int aprilTagNumber){
        limelight.pipelineSwitch(0); //pipeline 0 is AprilTag

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Access general information
                /*Pose3D botpose = result.getBotpose();
                Pose3D botPose_MT2 = result.getBotpose_MT2();

                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {

                    telemetry.addData("Botpose", botpose.toString());
                    //telemetry.addData("Botpose MT2",botPose_MT2.toString());

                    Position pos_in_Meter = botPose_MT2.getPosition();
                    Position pos_in_INCH = pos_in_Meter.toUnit(DistanceUnit.INCH);

                    double x = pos_in_INCH.x;
                    double y = pos_in_INCH.y;
                    double z = pos_in_INCH.z;

                    telemetry.addLine("XYZ " +
                            JavaUtil.formatNumber(pos_in_INCH.x, 6, 1) + " " +
                            JavaUtil.formatNumber(pos_in_INCH.y, 6, 1) + " " +
                            JavaUtil.formatNumber(pos_in_INCH.z, 6, 1) + "  (INCH)");

                    YawPitchRollAngles ypr = botPose_MT2.getOrientation();
                    double yawFromLimelight = ypr.getYaw();
                    telemetry.addLine("YAW " +
                            JavaUtil.formatNumber(yawFromLimelight, 6, 1) + "  (DEG)");


                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                 */

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                int id = fr.getFiducialId(); // The ID number of the fiducial
                double x = fr.getTargetXDegrees(); // Where it is (left-right)
                double y = fr.getTargetYDegrees(); // Where it is (up-down)

                /*
                getTargetPoseCameraSpace
                 Z +ve = distance of april tag with respect to the camera
                 X +ve = April tag is to the right of the camera
                 */
                Position pos_in_INCH2 = fr.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);

                //getCameraPoseTargetSpace also works
                //Position pos_in_INCH2 = fr.().getCameraPoseTargetSpace.getPosition().toUnit(DistanceUnit.INCH);
                //getRobotPoseTargetSpace -> did not try
                //Position pos_in_INCH2 = fr.getRobotPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);


                double y_distance = pos_in_INCH2.y;
                double x_distance = pos_in_INCH2.x;
                double z_distance = pos_in_INCH2.z;

                telemetry.addLine("getTargetPoseCameraSpace");
                telemetry.addLine("XYZ " +
                        JavaUtil.formatNumber(x_distance, 6, 1) + " " +
                        JavaUtil.formatNumber(y_distance, 6, 1) + " "  +
                        JavaUtil.formatNumber(z_distance, 6, 1) + " "  +"  (INCH)");

                YawPitchRollAngles ypr = fr.getCameraPoseTargetSpace().getOrientation();
                double yaw = ypr.getYaw(); //degrees?? - need to check

                telemetry.addLine("Yaw " +
                             JavaUtil.formatNumber(yaw, 6, 1) + " "  +"  (INCH)");





            }
            sleep(2000);

        }
        else {
            telemetry.addData("Limelight", "No data available");
        }
    }

    private void Init_IMU() {
        IMU.Parameters IMUParameter;

        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        IMUParameter = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // This will use IMU gyroscope and accelerometer
        // to calculate the relative orientation of the hub and thus the robot
        // Warn driver this make take several seconds
        telemetry.addData("Status", "Init IMU .... Please Wait");
        telemetry.update();
        // Initialize IMU using parameter object
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        imu_IMU.initialize(IMUParameter);
        telemetry.addData("Status", "IMU Initialized");
        telemetry.update();
    }
}