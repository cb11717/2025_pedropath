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
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility.Limelight3AAprilTag;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="StraightLine", group="Test", preselectTeleOp="ATHENS TwoCon_Teleop_1400 6nov25")
public class StraightLine extends OpMode {

    double shooterPower = 1.0;
    double shooterVelocityWall = 1600;

    Artifact artifact;
    Intake intake;
    Hood hood;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** Start Pose and Shoot pose is the same for our robot */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose finalPose = new Pose(28,0, Math.toRadians(0));

    private PathChain  strightLine;


    public void buildPaths(){

        strightLine = follower.pathBuilder()
                .addPath(new BezierLine(startPose, finalPose))
                .setConstantHeadingInterpolation(finalPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(strightLine, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    setPathState(-1);
                    break;
                }
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

        int aprilTagDetected = 21;
       /* artifact = new Artifact(hardwareMap,aprilTagDetected);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        hood.setHoodPosition(0.95);

        */

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



}
