package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility.ColorDetector;

//April Tag ID 21 - GPP
//April Tag ID 22 - PGP
//April Tag ID 23 - PPG

public class Artifact {

    CRServo ArtifactL;
    CRServo ArtifactC;
    CRServo ArtifactR;

    Kicker kicker;
    Shooter shooter;
    ColorDetector colorDetector;
    int aprilTagNumber;
    boolean artifactShootingCompete;
    double shooterPower;

    public Artifact(HardwareMap hardwareMap, int iAprilTagNumber){
        this.ArtifactL = hardwareMap.get(CRServo.class, "ArtifactL");
        this.ArtifactC = hardwareMap.get(CRServo.class, "ArtifactC");
        this.ArtifactR = hardwareMap.get(CRServo.class, "ArtifactR");

        this.ArtifactL.setDirection(CRServo.Direction.REVERSE);
        this.ArtifactC.setDirection(CRServo.Direction.REVERSE);
        this.ArtifactR.setDirection(CRServo.Direction.FORWARD);

        this.aprilTagNumber = iAprilTagNumber;
        this.artifactShootingCompete = true; //true is default
        this.shooterPower = 1.0;

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

    /*
     detect color at each location. If color is not detected, and distance is measurable,
     shoot irrespective of the color
     */
    public void shootArtifact(double iShooterPower){

        this.artifactShootingCompete = false;
        if(iShooterPower > 0) {
            this.shooterPower = iShooterPower;
        }

        this.shooter.shoot(this.shooterPower);

        //start artifact based color detected/distance detected and the AprilTag motif
        int colorLeft = this.colorDetector.detectColor("L");
        int colorCenter = this.colorDetector.detectColor("C");
        int colorRight = this.colorDetector.detectColor("R");

        //if April Tag ID 21, shooting sequence should be GPP
        if( this.aprilTagNumber == 21){
            if(colorLeft == 2){
                this.ArtifactL.setPower(1);
                this.sleep(100);
                this.ArtifactC.setPower(1);
                this.sleep(100);
                this.ArtifactR.setPower(1);

            } else if (colorCenter == 2 ){
                this.ArtifactC.setPower(1);
                this.sleep(100);
                this.ArtifactR.setPower(1);
                this.sleep(100);
                this.ArtifactL.setPower(1);

            } else if(colorRight == 2){
                this.ArtifactR.setPower(1);
                this.sleep(100);
                this.ArtifactL.setPower(1);
                this.sleep(100);
                this.ArtifactC.setPower(1);
            }
            //if any of the distance sensors still show that there is an artifact, just shoot
            //it
            this.shootUnidentifiableArtifact(colorLeft,colorCenter,colorRight );
        } else if (this.aprilTagNumber == 22){
            //April Tag ID 22 - shooting sequence should be PGP
            if(colorLeft == 3){
                this.ArtifactL.setPower(1);
                this.sleep(100);
                if(colorCenter == 2){
                    this.ArtifactC.setPower(1);
                    this.sleep(100);
                    this.ArtifactR.setPower(1);
                }else if(colorRight == 2){
                    this.ArtifactR.setPower(1);
                    this.sleep(100);
                    this.ArtifactC.setPower(1);
                }
            } else if(colorCenter == 3){
                this.ArtifactC.setPower(1);
                this.sleep(100);
                if(colorLeft == 2){
                    this.ArtifactL.setPower(1);
                    this.sleep(100);
                    this.ArtifactR.setPower(1);
                }else if(colorRight == 2){
                    this.ArtifactR.setPower(1);
                    this.sleep(100);
                    this.ArtifactL.setPower(1);
                }

            } else if( colorRight == 3){
                this.ArtifactR.setPower(1);
                this.sleep(100);
                if(colorLeft == 2){
                    this.ArtifactL.setPower(1);
                    this.sleep(100);
                    this.ArtifactC.setPower(1);
                }else if(colorCenter == 2){
                    this.ArtifactC.setPower(1);
                    this.sleep(100);
                    this.ArtifactL.setPower(1);
                }
            }
            //if any of the distance sensors still show that there is an artifact, just shoot
            //it
            this.shootUnidentifiableArtifact(colorLeft,colorCenter,colorRight );
        } else if (this.aprilTagNumber == 23){
            //if April Tag ID 23 - shooting sequence should be PPG
            if(colorLeft == 3){
                this.ArtifactL.setPower(1);
                this.sleep(100);
                if(colorRight == 3){
                    this.ArtifactR.setPower(1);
                    this.sleep(100);
                    this.ArtifactC.setPower(1);
                }else if(colorCenter == 3){
                    this.ArtifactC.setPower(1);
                    this.sleep(100);
                    this.ArtifactR.setPower(1);
                }
            } else if(colorRight == 3){
                this.ArtifactR.setPower(1);
                this.sleep(100);
                if(colorLeft == 3){
                    this.ArtifactL.setPower(1);
                    this.sleep(100);
                    this.ArtifactC.setPower(1);
                }else if(colorCenter == 3){
                    this.ArtifactC.setPower(1);
                    this.sleep(100);
                    this.ArtifactL.setPower(1);
                }
            } else if (colorCenter == 3){
                this.ArtifactC.setPower(1);
                this.sleep(100);
                if(colorRight == 3){
                    this.ArtifactR.setPower(1);
                    this.sleep(100);
                    this.ArtifactL.setPower(1);
                } else if(colorLeft == 3){
                    this.ArtifactL.setPower(1);
                    this.sleep(100);
                    this.ArtifactR.setPower(1);
                }
            }
            //if any of the distance sensors still show that there is an artifact, just shoot
            //it
            this.shootUnidentifiableArtifact(colorLeft,colorCenter,colorRight );
        } else {
            //did not get aprilTag, just shoot in any sequence
            this.ArtifactL.setPower(1);
            this.sleep(100);
            this.ArtifactC.setPower(1);
            this.sleep(100);
            this.ArtifactR.setPower(1);
        }
        this.kicker.run();
        this.sleep(100);

        this.kicker.stop();
        this.shooter.stop();
        this.stopArtifacts();

        this.artifactShootingCompete = true;

    }

    public void sleep(int milliseconds){
        try {
            // Pause execution for 3 seconds (3000 milliseconds)
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            // Handle the InterruptedException if the thread is interrupted while sleeping
            Thread.currentThread().interrupt(); // Re-interrupt the current thread

        }
    }

    public void stopArtifacts(){
        this.ArtifactL.setPower(0);
        this.ArtifactC.setPower(0);
        this.ArtifactR.setPower(0);
    }

   /*
   This function will shoot the artifacts if the color sensor is not able to detect the color,
   but the colorSensor V3 ( also a distance sensor) identifies an artifact. In such cases, the
   value of colorLeft, colorCenter and colorRight will be = 1
    */
    public void shootUnidentifiableArtifact(int colorLeft,int colorCenter,int colorRight)
    {

        if(colorLeft == 1){
            this.ArtifactL.setPower(1);
        }
        if (colorCenter == 1){
            this.ArtifactC.setPower(1);
        }
        if (colorRight == 1){
            this.ArtifactR.setPower(1);
        }

    }
}
