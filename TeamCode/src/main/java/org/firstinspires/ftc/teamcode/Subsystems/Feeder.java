package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Feeder {

    private DcMotorEx feedermotor;
    private DcMotorEx gatemotor;
    double feederSpeed = 1200;
    double gateSpeed = 2000;


    public Feeder(HardwareMap hwMap) {

        feedermotor = hwMap.get(DcMotorEx.class, "feeder");

        feedermotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feedermotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        feedermotor.setDirection(DcMotorEx.Direction.FORWARD);
        //        flywheel.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);
        feedermotor.setVelocity(feederSpeed);

        gatemotor = hwMap.get(DcMotorEx.class, "gate");

        gatemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gatemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gatemotor.setDirection(DcMotorEx.Direction.FORWARD);
        //        flywheel.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);
        gatemotor.setVelocity(gateSpeed);
    }
    public void feederOff() {
        feedermotor.setPower(0);
    }
    public void on() {
        feedermotor.setVelocity(feederSpeed);
    }
    public void gateoff() {
        gatemotor.setPower(0);
    }
    public void gateon() {
        gatemotor.setVelocity(gateSpeed);
    }

    public double getFeederSpeed() {
        return feedermotor.getVelocity();
    }
}
