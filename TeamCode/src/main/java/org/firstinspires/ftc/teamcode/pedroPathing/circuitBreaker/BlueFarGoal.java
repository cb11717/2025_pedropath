package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.ArtifactNew;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility.ColorDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility.Limelight3AAprilTag;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name="BlueFarGoal", group="Auto", preselectTeleOp="2Con_Tele_Cam LoLag 1925 11dec25")
@Disabled
public class BlueFarGoal extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    boolean isPickUp2Needed = true;
    double shooterPower = 1.0;
    double shooterVelocityFar = 1930; //2500
    double overRideShooterVelocityFar = 1870;
    Artifact artifact;
    Intake intake;
    Limelight3AAprilTag limelight;
    Hood hood;
    Shooter shooter;
    ColorDetector colorDetector;
    int aprilTagDetected = 21;
    int sleepTimer = 1500;

    int colorLeft, colorCenter, colorRight;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * This visualizer is very easy to use to find and create paths/pathchains/poses:
     * https://visualizer.pedropathing.com/
     *
     *  Pedro’s coordinate system spans an interval of [0, 144]
     * for Decode [0,0] is at the Red Human player zone ( bottom left corner of field)
     */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(56,14, Math.toRadians(109)); //Linear
    private final Pose controlPose1 = new Pose(37,30);
    private final Pose pickUpPose1 = new Pose(12,9, Math.toRadians(181)); //Linear
    private final Pose controlPose2 = new Pose(35,30);
    private final Pose shootPose2 = new Pose(54,21, Math.toRadians(112)); //Linear
    private final Pose intermediatePose2 = new Pose(29,24, Math.toRadians(90)); //Linear
    private final Pose pickUpPose2 = new Pose(29,31, Math.toRadians(90)); //Constant
    private final Pose shootPose3 = new Pose(53,20,Math.toRadians(113));
    private final Pose leavePose = new Pose (43, 31, Math.toRadians(110));
    /*private final Pose controlPose3 = new Pose(65,76);
    private final Pose shootPose3 = new Pose(56,105, Math.toRadians(120)); //Linear
     */

    private PathChain  shootArtifact1, grabPickup1, shootArtifact2, grabPickup2,
            ShootArtifact3, leaveLaunchLine;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        shootArtifact1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(1))
                .build();

        /*grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1,controlPose1,pickUpPose1))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), pickUpPose1.getHeading())
                .addParametricCallback(0, () -> {intake.run();})
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(5))
                .build();

         */

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1,controlPose1,pickUpPose1))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), pickUpPose1.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

        shootArtifact2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose1,controlPose2,shootPose2))
                .setLinearHeadingInterpolation(pickUpPose1.getHeading(), shootPose2.getHeading())
                .addParametricCallback(0, () -> {this.lightUpLED();})
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(1))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, intermediatePose2))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), intermediatePose2.getHeading())
                .addPath(new BezierLine(intermediatePose2, pickUpPose2))
                .setConstantHeadingInterpolation(pickUpPose2.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(1))
                .build();

        ShootArtifact3 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose2,shootPose3))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), shootPose3.getHeading())
                .addParametricCallback(0, () -> {this.lightUpLED();})
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(1))
                .build();

        leaveLaunchLine = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, leavePose))
                .setConstantHeadingInterpolation(leavePose.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

       /* ShootArtifact3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose2,controlPose3,shootPose3))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), shootPose3.getHeading())
                .build();

        */

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.run();
                shooter.start(shooterPower, overRideShooterVelocityFar);
                follower.followPath(shootArtifact1, true);
                setPathState(1);
                break;
            case 1:
                //if(!follower.isBusy() || (follower.getVelocity().getMagnitude() <= follower.getCurrentPath().getPathEndVelocityConstraint())){
                if(!follower.isBusy()){
                   artifact.shootArtifact(shooterPower, shooterVelocityFar,
                                           -1, -1, -1);
                    colorDetector.clearLedColor();
                    setPathState(2);

                }
                break;
            case 2:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup1, true);
                    hood.setHoodPosition(0.63);

                    setPathState(3);
                    //setPathState(-1);
                }

                break;
            case 3:
                if(!follower.isBusy()){
                    //intake.stop();
                   this.artifact.sleep(750);
                    follower.followPath(shootArtifact2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    artifact.shootArtifact(shooterPower, shooterVelocityFar,
                                           this.colorLeft, this.colorCenter, this.colorRight);
                    colorDetector.clearLedColor();
                    setPathState(5);
                }
                break;
            case 5:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup2, true);
                    if( isPickUp2Needed == true) {
                        setPathState(6);
                    } else{
                        setPathState(9);
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    //intake.stop();
                    this.artifact.sleep(750);
                    follower.followPath(ShootArtifact3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    artifact.shootArtifact(shooterPower, shooterVelocityFar,
                                           this.colorLeft, this.colorCenter, this.colorRight);
                    colorDetector.clearLedColor();
                    setPathState(8);
                }
                break;
            case 8:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(leaveLaunchLine, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    intake.stop();
                    this.limelight.stopLimelight();
                    shooter.stop();
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("AprilTag", this.aprilTagDetected);
        //robot’s current velocity
        //telemetry.addData("Velocity Magnitude", follower.getVelocity().getMagnitude());
        //target velocity the robot should have when it finishes the current path
        //telemetry.addData("PathEndVelocityConstraint", follower.getCurrentPath().getPathEndVelocityConstraint());
        telemetry.update();

    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        limelight = new Limelight3AAprilTag(hardwareMap);
        this.aprilTagDetected = limelight.getAprilTagNumber(0); // pipeline 0 is for Motif aprilTag

        int count = 0;

        while (this.aprilTagDetected != 21 && this.aprilTagDetected != 22 && this.aprilTagDetected != 23) {
            this.aprilTagDetected = limelight.getAprilTagNumber(0);

            telemetry.addData("AprilTag Detected", aprilTagDetected);
            telemetry.addData("Count", count);
            telemetry.update();

            if(count >5){
                break;
            }
            count++;
        }

        if (this.aprilTagDetected != 21 && this.aprilTagDetected != 22 && this.aprilTagDetected != 23) {
            this.aprilTagDetected = 21;
        }

        telemetry.addData("AprilTag Detected", aprilTagDetected);
        telemetry.update();

        artifact = new Artifact(hardwareMap,this.aprilTagDetected, this.sleepTimer, telemetry);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        //hood.setHoodPosition(0.63);
        hood.setHoodPosition(0.59);
        shooter = new Shooter(hardwareMap);
        colorDetector = new ColorDetector(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
         setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    private void lightUpLED(){
        this.colorLeft = this.colorDetector.detectColor("L");
        this.colorCenter = this.colorDetector.detectColor("C");
        this.colorRight = this.colorDetector.detectColor("R");
    }


}
