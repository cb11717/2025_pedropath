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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility.Limelight3AAprilTag;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="RedNearGoal", group="Auto")
public class RedNearGoal extends OpMode{

    boolean pickUp3Artifacts = true;
    double shooterPower = 1.0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    Artifact artifact;
    Intake intake;
    Limelight3AAprilTag limelight;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** Start Pose and Shoot pose is the same for our robot */
    private final Pose startPose = new Pose(128, 111, Math.toRadians(90));
    private final Pose intermediatePose1 = new Pose(120,100, Math.toRadians(-90)); //Linear
    private final Pose pickUpPose1 = new Pose (120, 90, Math.toRadians(-90)); //Constant
    private final Pose readMotifPose = new Pose(109,96, Math.toRadians(110)); //Linear
    private final Pose shootPose2 = new Pose(109,120, Math.toRadians(45)); //Linear
    private final Pose intermediatePose2 = new Pose(120,72, Math.toRadians(-90)); //Linear
    private final Pose pickUpPose2 = new Pose (120, 66, Math.toRadians(-90)); //Constant
    private final Pose ShootPose3 = new Pose (109, 120, Math.toRadians(45)); //Liner
    private final Pose intermediatePose3 = new Pose (120, 48, Math.toRadians(-90)); //Linear
    private final Pose pickUpPose3 = new Pose (120, 42, Math.toRadians(-90)); //Constant
    private final Pose ShootPose4 = new Pose (109, 120, Math.toRadians(45)); //Liner

    private Path scorePreload;
    private PathChain  grabPickup1, readMotif, shootArtifact2, grabPickup2, ShootArtifact3,
            grabPickup3, ShootArtifact4;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths(){

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intermediatePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intermediatePose1.getHeading())
                .addPath(new BezierLine(intermediatePose1, pickUpPose1))
                .setConstantHeadingInterpolation(pickUpPose1.getHeading())
                .addParametricCallback(0, () -> {intake.run();})
                .build();

        readMotif = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose1, readMotifPose))
                .setLinearHeadingInterpolation(pickUpPose1.getHeading(), readMotifPose.getHeading())
                .build();

        shootArtifact2 = follower.pathBuilder()
                .addPath(new BezierLine(readMotifPose, shootPose2))
                .setLinearHeadingInterpolation(readMotifPose.getHeading(), shootPose2.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, intermediatePose2))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), intermediatePose2.getHeading())
                .addPath(new BezierLine(intermediatePose2, pickUpPose2))
                .setConstantHeadingInterpolation(pickUpPose2.getHeading())
                .addParametricCallback(0, () -> {intake.run();})
                .build();

        ShootArtifact3 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose2, ShootPose3))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), ShootPose3.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose3, intermediatePose3))
                .setLinearHeadingInterpolation(ShootPose3.getHeading(), intermediatePose3.getHeading())
                .addPath(new BezierLine(intermediatePose3, pickUpPose3))
                .setConstantHeadingInterpolation(pickUpPose3.getHeading())
                .addParametricCallback(0, () -> {intake.run();})
                .build();

        ShootArtifact4 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose3, ShootPose4))
                .setLinearHeadingInterpolation(pickUpPose3.getHeading(), ShootPose4.getHeading())
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                //score preload, if artifact is not busy, go to next state
                artifact.shootArtifact(shooterPower);
                setPathState(1);
                break;
            case 1:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    intake.stop();
                    follower.followPath(readMotif, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    int aprilTagDetected = getMotifAprilTag();
                    artifact.setAprilTag(aprilTagDetected);

                    follower.followPath(shootArtifact2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    artifact.shootArtifact(shooterPower);
                    setPathState(5);
                }
                break;
            case 5:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    intake.stop();
                    follower.followPath(ShootArtifact3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    artifact.shootArtifact(shooterPower);
                    //if 2 artifact pickup, no more pickup needed, stop the path
                    if (pickUp3Artifacts == true){
                        setPathState(8);
                    } else {
                        setPathState(11);
                    }
                }
                break;
            case 8:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    intake.stop();
                    follower.followPath(ShootArtifact4, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    artifact.shootArtifact(shooterPower);
                    setPathState(11);
                }
                break;
            case 11:
                if(artifact.isArtifactShootingComplete()){
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */

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
        telemetry.update();

    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        limelight = new Limelight3AAprilTag(hardwareMap);
        int aprilTagDetected = getMotifAprilTag();

        artifact = new Artifact(hardwareMap,aprilTagDetected);
        intake = new Intake(hardwareMap);

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

    // Limelight pipeline 0 is configured for Motif aprilTag
    private int getMotifAprilTag(){
        int aprilTagDetected = this.limelight.getAprilTagNumber(0);

        telemetry.addData("AprilTag Detected", aprilTagDetected);
        telemetry.update();

        if (aprilTagDetected != 21 && aprilTagDetected != 22 && aprilTagDetected != 23) {
            aprilTagDetected = 21;
        }
        return aprilTagDetected;
    }
}
