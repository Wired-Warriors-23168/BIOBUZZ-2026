package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Feeder {

    private DcMotorEx feeder;
    double feederSpeed = 2000;

    public Feeder(HardwareMap hwMap) {

        feeder = hwMap.get(DcMotorEx.class, "feeder");

        feeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        feeder.setDirection(DcMotorEx.Direction.FORWARD);
        //        flywheel.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);
        feeder.setVelocity(1.0);
    }
    public void off() {
        feeder.setPower(0);
    }
    public void on() {
        feeder.setPower(1.0);
    }
}
