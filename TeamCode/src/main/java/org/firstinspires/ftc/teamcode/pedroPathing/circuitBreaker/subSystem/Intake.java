package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotor intake;

    public Intake(HardwareMap hardwareMap){
        this.intake = hardwareMap.get(DcMotor.class, "intake");

        this.intake.setDirection(DcMotor.Direction.REVERSE);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void run(){
        this.intake.setPower(1.0);
    }

    public void stop(){
        this.intake.setPower(0);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
