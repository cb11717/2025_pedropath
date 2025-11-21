package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDetector {

    ColorSensor ColorL; //ColorL is the LEFT side sensor as observed from back of the robot
    ColorSensor ColorC; //ColorC is the CENTER sensor
    ColorSensor ColorR; //ColorR is the RIGHT side sensor as observed from back of the robot

    // LED controls - green = 0.5, white = 1, red=.277, yellow=.388, blue=.611, purple=.722, orange=.333
    Servo rgbL;
    Servo rgbC;
    Servo rgbR;

    int gain;

    public ColorDetector(HardwareMap hardwareMap){
        ColorL = hardwareMap.get(ColorSensor.class, "ColorL");
        ColorC = hardwareMap.get(ColorSensor.class, "ColorC");
        ColorR = hardwareMap.get(ColorSensor.class, "ColorR");

        rgbL = hardwareMap.get(Servo .class, "rgbL");
        rgbC = hardwareMap.get(Servo .class, "rgbC");
        rgbR = hardwareMap.get(Servo .class, "rgbR");

        int gain = 4;

        ((NormalizedColorSensor) ColorL).setGain(gain);
        ((NormalizedColorSensor) ColorC).setGain(gain);
        ((NormalizedColorSensor) ColorR).setGain(gain);

    }

    /*
     Input: iArtifactPosition is the position of the artifact in the intake whose color needs to be
     identified. Expected value "L", "C", "R" where "L" represents Left artifact as observed from
     the back of the robot

     Output: integer value
             0: no artifact detected
             1: artifact detected, but color cannot be identified
             2: Green artifact
             3: Purple artifact
     */
    public int detectColor(String iArtifactPosition){
        int detectedColor = 0;

        int color;
        float hue;
        double distanceToArtifact;


        if ( iArtifactPosition.equals("L")){
            NormalizedRGBA normalizedColorsLeft = ((NormalizedColorSensor) ColorL).getNormalizedColors();

            color = normalizedColorsLeft.toColor();
            hue = JavaUtil.colorToHue(color);
            distanceToArtifact = ((DistanceSensor) ColorL).getDistance(DistanceUnit.CM);

            detectedColor = detectColorFromHueValue(hue, distanceToArtifact);

            if (detectedColor == 2){
                rgbL.setPosition(0.5);
            } else if (detectedColor == 3){
                rgbL.setPosition(0.72);
            } else {
                rgbL.setPosition(0);
            }

        } else if (iArtifactPosition.equals("C")){
            NormalizedRGBA normalizedColorCenter = ((NormalizedColorSensor) ColorC).getNormalizedColors();

            color = normalizedColorCenter.toColor();
            hue = JavaUtil.colorToHue(color);
            distanceToArtifact = ((DistanceSensor) ColorC).getDistance(DistanceUnit.CM);

            detectedColor = detectColorFromHueValue(hue, distanceToArtifact);

            if (detectedColor == 2){
                rgbC.setPosition(0.5);
            } else if (detectedColor == 3){
                rgbC.setPosition(0.72);
            } else {
                rgbC.setPosition(0);
            }

        } else { // iArtifactPosition = "R"
            NormalizedRGBA normalizedColorsRight = ((NormalizedColorSensor) ColorR).getNormalizedColors();

            color = normalizedColorsRight.toColor();
            hue = JavaUtil.colorToHue(color);
            distanceToArtifact = ((DistanceSensor) ColorR).getDistance(DistanceUnit.CM);

            detectedColor = detectColorFromHueValue(hue, distanceToArtifact);

            if (detectedColor == 2){
                rgbR.setPosition(0.5);
            } else if (detectedColor == 3){
                rgbR.setPosition(0.72);
            } else {
                rgbR.setPosition(0);
            }
        }

        return detectedColor;
    }

    public int detectColorFromHueValue(float hue, double distanceToArtifact){
        int detectedColor = 0;

        if (hue > 90 && hue <= 200) {
            //telemetry.addData(">>> COLOR", "Green");
            detectedColor = 2;
        } else if (hue > 200 && hue < 360) {
            // telemetry.addData(">>> COLOR", "Purple");
            detectedColor = 3;
        } else if (distanceToArtifact > 0 && distanceToArtifact < 3 ){
            //telemetry.addData(">>> COLOR", "artifact detected, but color cannot be identified");
            detectedColor = 1; //artifact is present in the intake but color sensor not able to detect the color
        } else {
            detectedColor = 0; //artifact not present in the intake
        }
        return detectedColor;
    }

    public void clearLedColor(){
        rgbC.setPosition(0);
        rgbR.setPosition(0);
        rgbL.setPosition(0);

    }


}
