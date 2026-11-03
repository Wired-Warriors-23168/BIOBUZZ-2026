package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private Servo hood;
    double pollenPos = 0;
    double nectarPos = 270;
    public Hood(HardwareMap hwMap) {
        hood = hwMap.get(Servo.class, "hood");
        hood.scaleRange(0, 270);
        hood.setDirection(Servo.Direction.FORWARD);
    }

    public Hood(HardwareMap hwMap, boolean isRamen) {
        hood = hwMap.get(Servo.class, "hood");
        hood.scaleRange(0, 270);
        if (isRamen) {
            hood.setDirection(Servo.Direction.FORWARD);
            pollenPos = 0;
            nectarPos = 270;
        } else {
            hood.setDirection(Servo.Direction.FORWARD);
            pollenPos = 0;
            nectarPos = 270;
        }
    }
    public void nectarPos() {
        hood.setPosition(nectarPos);
    }
    public void pollenPos() {
        hood.setPosition(pollenPos);
    }
    public double getHoodPos() {
        return hood.getPosition();
    }
}