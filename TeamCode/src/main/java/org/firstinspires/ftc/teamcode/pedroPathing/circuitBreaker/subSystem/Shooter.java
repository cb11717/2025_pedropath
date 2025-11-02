package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    DcMotor Shoot1;
    DcMotor Shoot2;
    DcMotor Shoot3;

    public Shooter(HardwareMap hardwareMap){

        this.Shoot1 = hardwareMap.get(DcMotor.class, "Shoot1");
        this.Shoot2 = hardwareMap.get(DcMotor.class, "Shoot2");
        this.Shoot3 = hardwareMap.get(DcMotor.class, "Shoot3");

        this.Shoot1.setDirection(DcMotor.Direction.REVERSE);
        this.Shoot2.setDirection(DcMotor.Direction.FORWARD);
        this.Shoot3.setDirection(DcMotor.Direction.REVERSE);

        this.Shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.Shoot1.setPower(0.0);
        this.Shoot2.setPower(this.Shoot1.getPower());
        this.Shoot3.setPower(this.Shoot1.getPower());

    }

    public void shoot(double shooterPower){
        this.Shoot1.setPower(shooterPower);
        this.Shoot2.setPower(this.Shoot1.getPower());
        this.Shoot3.setPower(this.Shoot1.getPower());
    }

    public void stop(){
        this.Shoot1.setPower(0.0);
        this.Shoot2.setPower(this.Shoot1.getPower());
        this.Shoot3.setPower(this.Shoot1.getPower());
        this.Shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
