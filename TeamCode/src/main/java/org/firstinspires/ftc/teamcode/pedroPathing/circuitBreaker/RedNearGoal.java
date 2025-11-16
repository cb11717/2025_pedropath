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

@Autonomous(name="RedNearGoal", group="Auto", preselectTeleOp="ATHENS TwoCon_Teleop_1400 6nov25")
public class RedNearGoal extends OpMode{

    boolean isPickUp2Needed = true;
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

    /** Start Pose and Shoot pose is the same for our robot */
    private final Pose startPose = new Pose(128, 111, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(111, 115, Math.toRadians(35));
    private final Pose intermediatePose1 = new Pose(118,100, Math.toRadians(-90));
    private final Pose pickUpPose1 = new Pose (118, 90, Math.toRadians(-90));// Constant
    private final Pose readMotifPose = new Pose(109,96, Math.toRadians(100)); //Linear
    private final Pose shootPose2 = new Pose (111,120,Math.toRadians(40));
    private final Pose intermediatePose2 = new Pose(115,72, Math.toRadians(-90));
    private final Pose controlPose2 = new Pose(100,80);
    private final Pose pickUpPose2 = new Pose (115,66, Math.toRadians(-90));//Constant
    private final Pose shootPose3 = new Pose (111,120,Math.toRadians(40));
    private final Pose controlPose3 =  new Pose (100,100);



    private Path scorePreload;
    private PathChain  shootArtifact1, grabPickup1, readMotif, shootArtifact2, grabPickup2, ShootArtifact3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths(){

        shootArtifact1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose1.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(3))
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, intermediatePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intermediatePose1.getHeading())
                .addPath(new BezierLine(intermediatePose1, pickUpPose1))
                .setConstantHeadingInterpolation(pickUpPose1.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(3))
                .build();

        readMotif = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose1, readMotifPose))
                .setLinearHeadingInterpolation(pickUpPose1.getHeading(), readMotifPose.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(3))
                .build();

        shootArtifact2 = follower.pathBuilder()
                .addPath(new BezierLine(readMotifPose, shootPose2))
                .setLinearHeadingInterpolation(readMotifPose.getHeading(), shootPose2.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(3))
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, controlPose2, intermediatePose2))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), intermediatePose2.getHeading())
                .addPath(new BezierLine(intermediatePose2, pickUpPose2))
                .setConstantHeadingInterpolation(pickUpPose2.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(3))
                .build();



        ShootArtifact3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose2, controlPose3, shootPose3))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), shootPose3.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(3))
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                intake.run();
                shooter.start(shooterPower, shooterVelocityWall);
                follower.followPath(shootArtifact1, true);
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
                    if( isPickUp2Needed == true) {
                        setPathState(6);
                    } else{
                        setPathState(9);
                    }
                }
                break;
            case 6:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup2, true);
                    setPathState(7);
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
        this.aprilTagDetected = getMotifAprilTag();

        artifact = new Artifact(hardwareMap,this.aprilTagDetected);
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
        return aprilTagDetected;
    }
}
