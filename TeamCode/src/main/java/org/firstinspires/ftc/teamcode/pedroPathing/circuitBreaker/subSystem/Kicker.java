package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Kicker {

    CRServo KickerL;
    CRServo KickerR;

    public Kicker(HardwareMap hardwareMap){
        this.KickerL = hardwareMap.get(CRServo.class, "KickerL");
        this.KickerR = hardwareMap.get(CRServo.class, "KickerR");

        KickerL.setDirection(CRServo.Direction.REVERSE);
        KickerR.setDirection(CRServo.Direction.FORWARD);
    }

    public void run(){
        this.KickerR.setPower(1.0);
        this.KickerL.setPower(this.KickerR.getPower());
    }

    public void stop(){
        this.KickerR.setPower(0.0);
        this.KickerL.setPower(0.0);
    }
}
