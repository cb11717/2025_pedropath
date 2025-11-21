package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility.Limelight3AAprilTag;

//example: https://pedropathing.com/docs/pathing/examples/auto


@Autonomous(name="BlueNearGoal", group="Auto", preselectTeleOp="ATHENS TwoCon_Teleop_1400 6nov25")
public class BlueNearGoal extends OpMode {

    boolean isShoot3Needed = false;
    double shooterPower = 1.0;
    double shooterVelocityWall = 1650;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    Artifact artifact;
    Intake intake;
    Limelight3AAprilTag limelight;
    Hood hood;
    Shooter shooter;
    int aprilTagDetected = 21;

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


    /** Start Pose and Shoot pose is the same for our robot */
    private final Pose startPose = new Pose(21, 123, Math.toRadians(140));

    private final Pose intermediatePose1 = new Pose(24,100,Math.toRadians(-90));
    private final Pose pickUpPose1 = new Pose (24,90, Math.toRadians(-90)); //Constant
    private final Pose readMotifPose = new Pose(35,96,Math.toRadians(45));
    private final Pose shootPose2 = new Pose(25,123,Math.toRadians(140));
    private final Pose intermediatePose2 = new Pose(24,72,Math.toRadians(-90));
    private final Pose controlPose2 = new Pose(40,80);
    private final Pose pickUpPose2 = new Pose (24,66, Math.toRadians(-90)); //Constant
    private final Pose shootPose3 = new Pose (25,123, Math.toRadians(140));
    private final Pose controlPose3  = new Pose (40,80);



    private Path scorePreload;
    private PathChain  shootArtifact1, grabPickup1, readMotif, shootArtifact2, grabPickup2, ShootArtifact3,
            grabPickup3, ShootArtifact4;

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

       grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intermediatePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intermediatePose1.getHeading())
                .addPath(new BezierLine(intermediatePose1, pickUpPose1))
                .setConstantHeadingInterpolation(pickUpPose1.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

        readMotif = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose1, readMotifPose))
                .setLinearHeadingInterpolation(pickUpPose1.getHeading(), readMotifPose.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

        shootArtifact2 = follower.pathBuilder()
                .addPath(new BezierLine(readMotifPose, shootPose2))
                .setLinearHeadingInterpolation(readMotifPose.getHeading(), shootPose2.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, controlPose2, intermediatePose2))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), intermediatePose2.getHeading())
                .addPath(new BezierLine(intermediatePose2, pickUpPose2))
                .setConstantHeadingInterpolation(pickUpPose2.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

        ShootArtifact3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose2, controlPose3, shootPose3))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), shootPose3.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.run();
                shooter.start(shooterPower, shooterVelocityWall);
                this.artifact.sleep(250);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy())
                {
                    artifact.shootArtifact(0.0, shooterVelocityWall);
                    setPathState(2);
                }
                break;
            case 2:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup1,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(readMotif, true);
                    setPathState(4);
                    //setPathState(-1);
                }
                break;
            case 4:
                int aprilTagDetected = getMotifAprilTag();
                if(!follower.isBusy()){
                    //intake.stop();
                    artifact.setAprilTag(aprilTagDetected);

                    follower.followPath(shootArtifact2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    artifact.shootArtifact(shooterPower, shooterVelocityWall);
                    setPathState(6);

                }
                break;
            case 6:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup2, true);
                    if( isShoot3Needed == true) {
                        setPathState(7);
                    } else{
                        setPathState(9);
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    // intake.stop();
                    follower.followPath(ShootArtifact3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    artifact.shootArtifact(shooterPower, shooterVelocityWall);
                    //if 2 artifact pickup, no more pickup needed, stop the path
                    setPathState(9);
                }
                break;
            case 9:
                if(artifact.isArtifactShootingComplete()){
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.addData("AprilTag", this.aprilTagDetected);
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
        hood = new Hood(hardwareMap);
        hood.setHoodPosition(0.95);
        shooter = new Shooter(hardwareMap);

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
        this.aprilTagDetected = this.limelight.getAprilTagNumber(0);

        telemetry.addData("AprilTag Detected", this.aprilTagDetected);
        telemetry.update();

        if (this.aprilTagDetected != 21 && this.aprilTagDetected != 22 && this.aprilTagDetected != 23) {
            this.aprilTagDetected = 21;
        }
        return this.aprilTagDetected;
    }
}

