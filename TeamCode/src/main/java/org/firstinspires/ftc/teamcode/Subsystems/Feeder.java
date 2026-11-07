package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Feeder {

    private DcMotorEx feedermotor;
    private DcMotorEx gatemotor;
    double feederSpeed = 1200;
    double gateSpeed = 800;
    double reverseSpeed = -1100;


    public Feeder(HardwareMap hwMap, boolean isRamen) {

        feedermotor = hwMap.get(DcMotorEx.class, "feeder");
        feedermotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feedermotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //        flywheel.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);

        gatemotor = hwMap.get(DcMotorEx.class, "gate");
        gatemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gatemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (isRamen) {
            feedermotor.setDirection(DcMotorEx.Direction.FORWARD);
            gatemotor.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            feedermotor.setDirection(DcMotorEx.Direction.REVERSE);
            gatemotor.setDirection(DcMotorEx.Direction.REVERSE);
        }
        //        flywheel.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);
    }

    public Feeder(HardwareMap hwMap) {

        feedermotor = hwMap.get(DcMotorEx.class, "feeder");
        feedermotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feedermotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //        flywheel.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);

        gatemotor = hwMap.get(DcMotorEx.class, "gate");
        gatemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gatemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //        flywheel.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);

            feedermotor.setDirection(DcMotorEx.Direction.FORWARD);
            gatemotor.setDirection(DcMotorEx.Direction.FORWARD);

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
    public void reverse() {
        gatemotor.setVelocity(reverseSpeed);
    }
    public double getFeederSpeed() {
        return feedermotor.getVelocity();
    }
}
