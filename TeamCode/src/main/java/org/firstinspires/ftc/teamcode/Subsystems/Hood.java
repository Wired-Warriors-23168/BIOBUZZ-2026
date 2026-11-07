package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private Servo hood;
    double pollenPos = 0;
    public double nectarPos = 0;
    double downPos = 0;
    public Hood(HardwareMap hwMap) {
        hood = hwMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.FORWARD);
    }

    public Hood(HardwareMap hwMap, boolean isRamen) {
        hood = hwMap.get(Servo.class, "hood");
        if (isRamen) {
            hood.setDirection(Servo.Direction.FORWARD);
            downPos = 0.02;
            pollenPos = 0.50;
            nectarPos = 0.1;
        } else {
            hood.setDirection(Servo.Direction.FORWARD);
            downPos = 0.28;
            pollenPos = downPos + 0.48;
            nectarPos = downPos + 0.16;
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