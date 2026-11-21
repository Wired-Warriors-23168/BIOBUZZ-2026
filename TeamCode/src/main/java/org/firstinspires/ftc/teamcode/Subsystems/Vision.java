package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Vision {
    public Limelight3A limelight;

    double tx = 0;
    double distance;

    int closepipeline = 0;
    int farpipeline = 1;
    public Vision(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public double getTx() {
        return tx;
    }

    public void aimClose() {
        limelight.pipelineSwitch(closepipeline);
    }

     public void aimFar() {
         limelight.pipelineSwitch(farpipeline);
     }

    public void update() {
        LLResult result = limelight.getLatestResult();
        tx = result.getTx();

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            tag.getCameraPoseTargetSpace();
        }

    }
}
