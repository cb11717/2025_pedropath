package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility.ColorDetector;


//April Tag ID 21 - GPP
//April Tag ID 22 - PGP
//April Tag ID 23 - PPG

public class ArtifactNew {

    CRServo ArtifactL;
    CRServo ArtifactC;
    CRServo ArtifactR;

    Kicker kicker;
    Shooter shooter;
    ColorDetector colorDetector;
    int aprilTagNumber;
    boolean artifactShootingCompete;
    double shooterPower;
    double shooterVelocity;
    int sleepTimer;
    Telemetry telemetry;

    public ArtifactNew(HardwareMap hardwareMap, int iAprilTagNumber, int iSleepTimer, Telemetry iTelemetry){
        this.ArtifactL = hardwareMap.get(CRServo.class, "ArtifactL");
        this.ArtifactC = hardwareMap.get(CRServo.class, "ArtifactC");
        this.ArtifactR = hardwareMap.get(CRServo.class, "ArtifactR");

        this.ArtifactL.setDirection(CRServo.Direction.REVERSE);
        this.ArtifactC.setDirection(CRServo.Direction.REVERSE);
        this.ArtifactR.setDirection(CRServo.Direction.FORWARD);

        this.aprilTagNumber = iAprilTagNumber;
        this.artifactShootingCompete = true; //true is default
        this.shooterPower = 1.0;
        this.shooterVelocity = 0.0;
        telemetry = iTelemetry;

        if(iSleepTimer > 0)
        {
            this.sleepTimer = iSleepTimer;
        } else {
            this.sleepTimer = 1500;
        }


        this.kicker = new Kicker(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.colorDetector = new ColorDetector(hardwareMap);
    }

    public boolean isArtifactShootingComplete(){
        return this.artifactShootingCompete;
    }

    public void setAprilTag(int iAprilTagNumber){
        this.aprilTagNumber = iAprilTagNumber;
    }

    public void sleep(int milliseconds){
        try {
            // Pause execution for 3 seconds (3000 milliseconds)
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            // Handle the InterruptedException if the thread is interrupted while sleeping
            //Thread.currentThread().interrupt(); // Re-interrupt the current thread

        }
    }

    public void stopArtifacts(){
        this.ArtifactL.setPower(0);
        this.ArtifactC.setPower(0);
        this.ArtifactR.setPower(0);
    }

    public void shootUnidentifiableArtifact(int colorLeft,int colorCenter,int colorRight) {
        this.shootArtifactC();
        this.shootArtifactR();
        this.shootArtifactL();
    }

    public void shootArtifactL() {

        this.ArtifactL.setPower(1);
        this.ArtifactR.setPower(-1);
        this.ArtifactC.setPower(-1);
        this.sleep(this.sleepTimer);
        this.ArtifactL.setPower(0);
        this.ArtifactR.setPower(0);
        this.ArtifactC.setPower(0);
    }

    public void shootArtifactC(){
        this.ArtifactC.setPower(1);
        this.ArtifactR.setPower(-1);
        this.ArtifactL.setPower(-1);
        this.sleep(this.sleepTimer);
        this.ArtifactC.setPower(0);
        this.ArtifactR.setPower(0);
        this.ArtifactL.setPower(0);

    }
    public void shootArtifactR(){
        this.ArtifactR.setPower(1);
        this.ArtifactC.setPower(-1);
        this.ArtifactL.setPower(-1);
        this.sleep(this.sleepTimer);
        this.ArtifactR.setPower(0);
        this.ArtifactC.setPower(0);
        this.ArtifactL.setPower(0);
    }

    public void shootArtifact(double iShooterPower, double iShooterVelocity,
                              int iColorL, int iColorC, int iColorR){

        this.artifactShootingCompete = false;

        if(iShooterVelocity > 0)
        {
            this.shooterVelocity = iShooterVelocity;
        }
        else if (iShooterPower > 0){
            this.shooterPower = iShooterPower;
        } else {
            this.shooterPower = 1.0;
        }

        this.kicker.run();

        //start artifact based color detected/distance detected and the AprilTag motif
        int colorLeft = iColorL;
        int colorCenter = iColorC;
        int colorRight = iColorR;

        if( iColorL == -1){
            //the colors have not yet been detected - perhaps beginning of the game
            /* since our selectors have an issue with shooting first set of artifacts,
            shoot in a particular order irrespective of motif */

            colorLeft = this.colorDetector.detectColor("L");
            colorCenter = this.colorDetector.detectColor("C");
            colorRight = this.colorDetector.detectColor("R");

            this.shootUnidentifiableArtifact(colorLeft,colorCenter,colorRight );

            this.kicker.stop();
            //this.shooter.stop();
            this.stopArtifacts();

            this.artifactShootingCompete = true;

            return;

        } else {
            boolean isShootingAttempted = false;
            if( this.aprilTagNumber == 21){

                if(colorLeft == 2){
                    this.shootArtifactL();
                    this.shootArtifactC();
                    this.shootArtifactR();

                    isShootingAttempted = true;

                } else if (colorCenter == 2 ){
                    this.shootArtifactC();
                    this.shootArtifactR();
                    this.shootArtifactL();

                    isShootingAttempted = true;

                } else if(colorRight == 2){
                    this.shootArtifactR();
                    this.shootArtifactL();
                    this.shootArtifactC();

                    isShootingAttempted = true;
                }
                //if shooting has not been attempted, blindly shoot all 3 artifacts
                if(isShootingAttempted == false) {
                    this.shootUnidentifiableArtifact(colorLeft, colorCenter, colorRight);
                    isShootingAttempted = true;
                }
            } else if (this.aprilTagNumber == 22){
                //April Tag ID 22 - shooting sequence should be PGP
                if(colorLeft == 3){
                    this.shootArtifactL();

                    if(colorCenter == 2){
                        this.shootArtifactC();
                        this.shootArtifactR();
                        isShootingAttempted = true;

                    }else if(colorRight == 2){
                        this.shootArtifactR();
                        this.shootArtifactC();
                        isShootingAttempted = true;

                    } else if(isShootingAttempted == false){
                        this.shootArtifactC();
                        this.shootArtifactR();
                        isShootingAttempted = true;

                    }
                } else if(colorCenter == 3){
                    this.shootArtifactC();

                    if(colorLeft == 2){
                        this.shootArtifactL();
                        this.shootArtifactR();
                        isShootingAttempted = true;

                    }else if(colorRight == 2){
                        this.shootArtifactR();
                        this.shootArtifactL();
                        isShootingAttempted = true;

                    }else if(isShootingAttempted == false){
                        this.shootArtifactR();
                        this.shootArtifactL();
                        isShootingAttempted = true;
                    }

                } else if( colorRight == 3){
                    this.shootArtifactR();

                    if(colorLeft == 2){
                        this.shootArtifactL();
                        this.shootArtifactC();
                        isShootingAttempted = true;

                    }else if(colorCenter == 2){
                        this.shootArtifactC();
                        this.shootArtifactL();
                        isShootingAttempted = true;

                    }else if(isShootingAttempted == false){
                        this.shootArtifactC();
                        this.shootArtifactL();
                        isShootingAttempted = true;
                    }
                }
                //if any of the distance sensors still show that there is an artifact, just shoot
                //it
                if(isShootingAttempted == false){
                    this.shootUnidentifiableArtifact(colorLeft,colorCenter,colorRight );
                    isShootingAttempted = true;
                }

            } else if (this.aprilTagNumber == 23){
                //if April Tag ID 23 - shooting sequence should be PPG
                if(colorLeft == 3){
                    this.shootArtifactL();

                    if(colorRight == 3){
                        this.shootArtifactR();
                        this.shootArtifactC();
                        isShootingAttempted = true;

                    }else if(colorCenter == 3){
                        this.shootArtifactC();
                        this.shootArtifactR();
                        isShootingAttempted = true;

                    }else if(isShootingAttempted == false){
                        this.shootArtifactC();
                        this.shootArtifactL();
                        isShootingAttempted = true;
                    }
                } else if(colorRight == 3){
                    this.shootArtifactR();

                    if(colorLeft == 3){
                        this.shootArtifactL();
                        this.shootArtifactC();
                        isShootingAttempted = true;

                    }else if(colorCenter == 3){
                        this.shootArtifactC();
                        this.shootArtifactL();
                        isShootingAttempted = true;

                    }else if(isShootingAttempted == false){
                        this.shootArtifactC();
                        this.shootArtifactL();
                        isShootingAttempted = true;
                    }
                } else if (colorCenter == 3){
                    this.shootArtifactC();

                    if(colorRight == 3){
                        this.shootArtifactR();
                        this.shootArtifactL();
                        isShootingAttempted = true;

                    } else if(colorLeft == 3){
                        this.shootArtifactL();
                        this.shootArtifactR();
                        isShootingAttempted = true;

                    } else if(isShootingAttempted == false){
                        this.shootArtifactC();
                        this.shootArtifactL();
                        isShootingAttempted = true;
                    }
                }
                //if any of the distance sensors still show that there is an artifact, just shoot
                //it
                if(isShootingAttempted == false){
                    this.shootUnidentifiableArtifact(colorLeft,colorCenter,colorRight );
                    isShootingAttempted = true;
                }
            } else {
                //did not get aprilTag, just shoot in any sequence
                this.shootArtifactC();
                this.shootArtifactL();
                this.shootArtifactR();

            }


            this.kicker.stop();
            this.artifactShootingCompete = true;
        }


    }
}
