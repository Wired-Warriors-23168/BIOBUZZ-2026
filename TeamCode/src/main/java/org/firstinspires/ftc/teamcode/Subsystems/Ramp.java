package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ramp {
    private Servo rampLeft;
    private Servo rampRight;
    double rampPosMin = 0;
    double rampPosMax = 1;
    double rampDownPos = 0.28;

    public Ramp(HardwareMap hwMap) {
        rampLeft = hwMap.get(Servo.class, "rampleft");
        rampLeft.setDirection(Servo.Direction.FORWARD);
        rampRight = hwMap.get(Servo.class, "rampright");
        rampRight.setDirection(Servo.Direction.FORWARD);
    }

    public Ramp(HardwareMap hwMap, boolean isRamen) {
        rampRight = hwMap.get(Servo.class, "rampright");
        rampLeft = hwMap.get(Servo.class, "rampleft");
        if (isRamen) {

        } else {

        }
    }
    public void setRampPosUp() {
        rampRight.setPosition(1);
        rampLeft.setPosition(0);
    }
    public void setRampPosDown() {
        rampRight.setPosition(1-rampDownPos);
        rampLeft.setPosition(rampDownPos);
    }
    public double getRampPos() {
        return rampLeft.getPosition();
    }
}