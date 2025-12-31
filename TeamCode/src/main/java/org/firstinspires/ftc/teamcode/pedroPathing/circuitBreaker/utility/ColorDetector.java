package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDetector {

    ColorSensor ColorL, ColorL2; //ColorL and L2 is the LEFT side sensor as observed from back of the robot
    ColorSensor ColorC, ColorC2; //ColorC and C2 is the CENTER sensor
    ColorSensor ColorR, ColorR2; //ColorR and R2 is the RIGHT side sensor as observed from back of the robot

    // LED controls - green = 0.5, white = 1, red=.277, yellow=.388, blue=.611, purple=.722, orange=.333
    Servo rgbL;
    Servo rgbC;
    Servo rgbR;

    int gain;

    public ColorDetector(HardwareMap hardwareMap){
        ColorL = hardwareMap.get(ColorSensor.class, "ColorL");
        ColorC = hardwareMap.get(ColorSensor.class, "ColorC");
        ColorR = hardwareMap.get(ColorSensor.class, "ColorR");

        ColorL2 = hardwareMap.get(ColorSensor.class, "ColorL2");
        ColorC2 = hardwareMap.get(ColorSensor.class, "ColorC2");
        ColorR2 = hardwareMap.get(ColorSensor.class, "ColorR2");

        rgbL = hardwareMap.get(Servo .class, "rgbL");
        rgbC = hardwareMap.get(Servo .class, "rgbC");
        rgbR = hardwareMap.get(Servo .class, "rgbR");

        float gain = (float)4;
        //int gain2 = 2;

        ((NormalizedColorSensor) ColorL).setGain(gain);
        ((NormalizedColorSensor) ColorC).setGain(gain);
        ((NormalizedColorSensor) ColorR).setGain(gain);


        /*
        * REV color sensors have a default gain of 3. L2/C2/R2 have a default gain
        * in this program
        */

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

        int color, color2;
        float hue, hue2;
        double distanceToArtifact, distanceToArtifact2;


        if ( iArtifactPosition.equals("L")){
            NormalizedRGBA normalizedColorsLeft = ((NormalizedColorSensor) ColorL).getNormalizedColors();
            NormalizedRGBA normalizedColorsLeft2 = ((NormalizedColorSensor) ColorL2).getNormalizedColors();

            color = normalizedColorsLeft.toColor();
            hue = JavaUtil.colorToHue(color);
            distanceToArtifact = ((DistanceSensor) ColorL).getDistance(DistanceUnit.CM);

            color2 = normalizedColorsLeft2.toColor();
            hue2 = JavaUtil.colorToHue(color2);
            distanceToArtifact2 = ((DistanceSensor) ColorL2).getDistance(DistanceUnit.CM);

            detectedColor = detectColorFromHueValue(hue,hue2, distanceToArtifact, distanceToArtifact2);


            if (detectedColor == 2 ){
                rgbL.setPosition(0.5);
            } else if (detectedColor == 3 ){
                rgbL.setPosition(0.72);
            } else {
                rgbL.setPosition(0);
            }

        } else if (iArtifactPosition.equals("C")){
            NormalizedRGBA normalizedColorCenter = ((NormalizedColorSensor) ColorC).getNormalizedColors();
            NormalizedRGBA normalizedColorsCenter2 = ((NormalizedColorSensor) ColorC2).getNormalizedColors();

            color = normalizedColorCenter.toColor();
            hue = JavaUtil.colorToHue(color);
            distanceToArtifact = ((DistanceSensor) ColorC).getDistance(DistanceUnit.CM);

            color2 = normalizedColorsCenter2.toColor();
            hue2 = JavaUtil.colorToHue(color2);
            distanceToArtifact2 = ((DistanceSensor) ColorC2).getDistance(DistanceUnit.CM);



            detectedColor = detectColorFromHueValue(hue, hue2, distanceToArtifact, distanceToArtifact2);

            if (detectedColor == 2){
                rgbC.setPosition(0.5);
            } else if (detectedColor == 3){
                rgbC.setPosition(0.72);
            } else {
                rgbC.setPosition(0);
            }

        } else { // iArtifactPosition = "R"
            NormalizedRGBA normalizedColorsRight = ((NormalizedColorSensor) ColorR).getNormalizedColors();
            NormalizedRGBA normalizedColorsRight2 = ((NormalizedColorSensor) ColorR2).getNormalizedColors();


            color = normalizedColorsRight.toColor();
            hue = JavaUtil.colorToHue(color);
            distanceToArtifact = ((DistanceSensor) ColorR).getDistance(DistanceUnit.CM);

            color2 = normalizedColorsRight2.toColor();
            hue2 = JavaUtil.colorToHue(color2);
            distanceToArtifact2 = ((DistanceSensor) ColorR2).getDistance(DistanceUnit.CM);


            detectedColor = detectColorFromHueValue(hue, hue2, distanceToArtifact,distanceToArtifact2 );

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

    public int detectColorFromHueValue(float hue, float hue2,
                                       double distance, double distance2){
        int detectedColor = 0;

        if (hue > 90 && hue < 180 && distance < 3|| hue2 > 90 && hue2 < 180 && distance2 < 3) {
            //telemetry.addData(">>> COLOR", "Green");
            detectedColor = 2;
        } else if (hue >= 180 && hue < 360 && distance < 3 || hue2 >= 180 && hue2 < 360 && distance2 < 3) {
            // telemetry.addData(">>> COLOR", "Purple");
            detectedColor = 3;
        } else if (distance > 0 && distance < 3 || distance2 > 0 && distance2 < 3){
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
