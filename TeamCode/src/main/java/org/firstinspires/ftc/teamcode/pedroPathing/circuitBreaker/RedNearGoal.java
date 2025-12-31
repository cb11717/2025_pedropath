package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility.ColorDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility.Limelight3AAprilTag;

@Autonomous(name="RedNearGoal", group="Auto", preselectTeleOp="2Con_Tele_Cam LoLag 1925 11dec25")
public class RedNearGoal extends OpMode{

    boolean isShoot3Needed = true;
    double shooterPower = 1.0;
    double shooterVelocityWall = 1325; //1700;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    Artifact artifact;
    Intake intake;
    Limelight3AAprilTag limelight;
    Hood hood;
    Shooter shooter;
    ColorDetector colorDetector;
    int aprilTagDetected = 21;
    int sleepTimer = 1550;

    int colorLeft, colorCenter, colorRight;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** Start Pose and Shoot pose is the same for our robot */
    //private final Pose startPose = new Pose(128, 111, Math.toRadians(90));
    //private final Pose controlPose1  = new Pose (117,117);
    private final Pose shootPose1 = new Pose(121, 123, Math.toRadians(35));
    private final Pose intermediatePose1 = new Pose(116,100, Math.toRadians(-90));
    private final Pose pickUpPose1 = new Pose (116, 90, Math.toRadians(-90));// Constant
    private final Pose readMotifPose = new Pose(109,96, Math.toRadians(100)); //Linear
    private final Pose shootPose2 = new Pose (122,123,Math.toRadians(40));
    private final Pose intermediatePose2 = new Pose(112,80, Math.toRadians(-90));
    private final Pose controlPose2 = new Pose(90,85);
    private final Pose pickUpPose2 = new Pose (112,66, Math.toRadians(-90));//Constant
    private final Pose shootPose3 = new Pose (121,123,Math.toRadians(40));
    private final Pose controlPose3 =  new Pose (100,100);
    private final Pose leavePose = new Pose (113, 66, Math.toRadians(40));



    private Path scorePreload;
    private PathChain  shootArtifact1, grabPickup1, readMotif, shootArtifact2,
            grabPickup2, ShootArtifact3, leaveLaunchLine;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths(){

       /* shootArtifact1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,controlPose1,shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose1.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

        */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, intermediatePose1))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), intermediatePose1.getHeading())
                .addPath(new BezierLine(intermediatePose1, pickUpPose1))
                .setConstantHeadingInterpolation(pickUpPose1.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

        readMotif = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose1, readMotifPose))
                .setLinearHeadingInterpolation(pickUpPose1.getHeading(), readMotifPose.getHeading())
                .addParametricCallback(0, () -> {this.lightUpLED();})
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
                .setHeadingConstraint(Math.toRadians(1))
                .build();



        ShootArtifact3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose2, controlPose3, shootPose3))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), shootPose3.getHeading())
                .addParametricCallback(0, () -> {this.lightUpLED();})
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
                .build();

        leaveLaunchLine = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, leavePose))
                .setConstantHeadingInterpolation(leavePose.getHeading())
                .setTranslationalConstraint(1.0)          // inches
                .setHeadingConstraint(Math.toRadians(2))
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
               // this.artifact.sleep(15);
                setPathState(1);
                break;
            case 1:
                artifact.shootArtifact(0.0, shooterVelocityWall,
                                       -1, -1, -1);
                colorDetector.clearLedColor();
                setPathState(2);
                break;
            case 2:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup1,true);
                    hood.setHoodPosition(0.92);
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
                    artifact.shootArtifact(shooterPower, shooterVelocityWall,
                                           this.colorLeft, this.colorCenter, this.colorRight);
                    colorDetector.clearLedColor();
                    setPathState(6);

                }
                break;
            case 6:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(grabPickup2, true);
                    if( isShoot3Needed == true) {
                        setPathState(7);
                    } else{
                        setPathState(10);
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
                    artifact.shootArtifact(shooterPower, shooterVelocityWall,
                                           this.colorLeft, this.colorCenter, this.colorRight);
                    colorDetector.clearLedColor();
                    setPathState(9);
                }
                break;
            case 9:
                if(artifact.isArtifactShootingComplete()){
                    follower.followPath(leaveLaunchLine, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
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

        artifact = new Artifact(hardwareMap,this.aprilTagDetected, this.sleepTimer, telemetry);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        hood.setHoodPosition(0.938);
        shooter = new Shooter(hardwareMap);
        colorDetector = new ColorDetector(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setStartingPose(shootPose1);

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

        if (this.aprilTagDetected != 21 && this.aprilTagDetected != 22 && this.aprilTagDetected != 23) {
            this.aprilTagDetected = 21;
        }

        telemetry.addData("AprilTag Detected", this.aprilTagDetected);
        telemetry.update();

        return aprilTagDetected;
    }

    private void lightUpLED(){
        this.colorLeft = this.colorDetector.detectColor("L");
        this.colorCenter = this.colorDetector.detectColor("C");
        this.colorRight = this.colorDetector.detectColor("R");
    }

}
