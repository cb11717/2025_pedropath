package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDetector {

    private ColorSensor ColorL; //ColorL is the LEFT side sensor as observed from back of the robot
    private ColorSensor ColorC; //ColorC is the CENTER sensor
    private ColorSensor ColorR; //ColorR is the RIGHT side sensor as observed from back of the robot

    int gain;

    public ColorDetector(HardwareMap hardwareMap){
        ColorL = hardwareMap.get(ColorSensor.class, "ColorL");
        ColorC = hardwareMap.get(ColorSensor.class, "ColorC");
        ColorR = hardwareMap.get(ColorSensor.class, "ColorR");

        int gain = 4;

        ((NormalizedColorSensor) ColorL).setGain(gain);
        ((NormalizedColorSensor) ColorC).setGain(gain);
        ((NormalizedColorSensor) ColorR).setGain(gain);

    }

    /*
     Input: iArtifactPosition is the position of the artifact in the intake who color needs to be
     identified

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

        if ( iArtifactPosition.equals("L")){
            NormalizedRGBA normalizedColorsLeft = ((NormalizedColorSensor) ColorL).getNormalizedColors();

            color = normalizedColorsLeft.toColor();;
            hue = JavaUtil.colorToHue(color);;

        } else if (iArtifactPosition.equals("C")){
            NormalizedRGBA normalizedColorCenter = ((NormalizedColorSensor) ColorC).getNormalizedColors();

            color = normalizedColorCenter.toColor();
            hue = JavaUtil.colorToHue(color);

        } else { // iArtifactPosition = "R"
            NormalizedRGBA normalizedColorsRight = ((NormalizedColorSensor) ColorR).getNormalizedColors();

            color = normalizedColorsRight.toColor();
            hue = JavaUtil.colorToHue(color);

        }

        return detectedColor;
    }
}
