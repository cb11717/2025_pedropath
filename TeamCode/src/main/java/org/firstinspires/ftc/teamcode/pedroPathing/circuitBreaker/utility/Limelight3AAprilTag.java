package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker.utility;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

 import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class Limelight3AAprilTag {

    private Limelight3A limelight;
    private Telemetry telemetry;

    public Limelight3AAprilTag(HardwareMap hardwareMap) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // This sets how often we ask Limelight for data (100 times per second)
        this.limelight.setPollRateHz(100);

        this.limelight.start();

    }

    public void stopLimelight() {
        this.limelight.stop();
    }

    /*
    returns the AprilTag number detected given a pipeline
    in Decode :
        - pipeline 0 is mapped to Motif
        - pipeline 1 is mapped to Blue Goal
        - pipeline 2 is mapped to Red Goal
     */
    public int getAprilTagNumber(int iLimelightPipeline) {
        this.limelight.pipelineSwitch(iLimelightPipeline);

        LLResult result = this.limelight.getLatestResult();

        if (result != null) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int id = fr.getFiducialId();

                return id; //return the first detected id
            }
        } else {
            return 0;
        }
        return 0;
    }
}
