package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.subSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {

    Servo HoodL;

    public Hood(HardwareMap hardwareMap){
        this.HoodL = hardwareMap.get(Servo.class, "HoodL");
    }

    //servoPosition can range from 0 to 1
    //servoPosition = 1 is Hood fully down
    public void setHoodPosition(int servoPosition){
        this.HoodL.setPosition(servoPosition);
    }
}
