package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    DcMotor Shoot1;
    DcMotor Shoot2;
    DcMotor Shoot3;

       double shooter_gear_ratio = 1.23;
    int Shooter_wheel_diameter = 75;
    int ticksPerSec = 1800;

    public Shooter(HardwareMap hardwareMap){

        this.Shoot1 = hardwareMap.get(DcMotor.class, "Shoot1");
        this.Shoot2 = hardwareMap.get(DcMotor.class, "Shoot2");
        this.Shoot3 = hardwareMap.get(DcMotor.class, "Shoot3");

        this.Shoot1.setDirection(DcMotor.Direction.REVERSE);
        this.Shoot2.setDirection(DcMotor.Direction.FORWARD);
        this.Shoot3.setDirection(DcMotor.Direction.REVERSE);

        this.Shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.Shoot2.setPower(0.0);
        this.Shoot1.setPower(this.Shoot2.getPower());
        this.Shoot3.setPower(this.Shoot2.getPower());

    }

    public void start(double shooterPower, double shooterVelocity){

        if(shooterVelocity > 0){
            ((DcMotorEx) this.Shoot2).setVelocity(shooterVelocity);
            ((DcMotorEx) this.Shoot1).setVelocity(shooterVelocity);
            ((DcMotorEx) this.Shoot3).setVelocity(shooterVelocity);

        } else{
            this.Shoot2.setPower(shooterPower);
            this.Shoot1.setPower(this.Shoot2.getPower());
            this.Shoot3.setPower(this.Shoot2.getPower());
        }


    }

    public void stop(){
        this.Shoot2.setPower(0.0);
        this.Shoot1.setPower(this.Shoot2.getPower());
        this.Shoot3.setPower(this.Shoot2.getPower());
        this.Shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getVelocity(){
        double shooterCurrentVelocity = ((DcMotorEx) this.Shoot2).getVelocity();
        return shooterCurrentVelocity;
    }
}
