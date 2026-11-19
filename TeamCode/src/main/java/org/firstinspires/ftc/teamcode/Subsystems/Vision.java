package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Vision {
    public Limelight3A limelight;

    double tx = 0;
    double distance;

    public Vision(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public double getTx() {
        return tx;
    }

    String tags = "";
    public void update() {
        LLResult result = limelight.getLatestResult();
        tx = result.getTx();

        tags = "";
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            tag.getCameraPoseTargetSpace();
            tags = tags + tag.getFiducialId() + " ";
        }

    }

    public String tagIDs() {
        return tags;
    }
}
