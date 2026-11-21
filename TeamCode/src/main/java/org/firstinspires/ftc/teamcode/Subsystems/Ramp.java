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
        rampLeft = hwMap.get(Servo.class, "leftRamp");
        rampLeft.setDirection(Servo.Direction.FORWARD);
        rampRight = hwMap.get(Servo.class, "rightRamp");
        rampRight.setDirection(Servo.Direction.FORWARD);
    }

    public Ramp(HardwareMap hwMap, boolean isRamen) {
        rampRight = hwMap.get(Servo.class, "rightRamp");
        rampRight.setDirection(Servo.Direction.FORWARD);
        rampLeft = hwMap.get(Servo.class, "leftRamp");
        rampLeft.setDirection(Servo.Direction.REVERSE);

        if (isRamen) {

        } else {

        }
    }
    public void setRampPosUp() {
        rampRight.setPosition(0);
        rampLeft.setPosition(0);
    }
    public void setRampPosDown() {
        rampRight.setPosition(rampDownPos);
        rampLeft.setPosition(rampDownPos);
    }
    public double getRampPos() {
        return rampLeft.getPosition();
    }
}