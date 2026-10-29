package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class  Intake{

    private DcMotorEx intake;
    private CRServo left;
    private CRServo right;
    double intakeSpeed = 4500;

    public Intake(HardwareMap hwMap) {

        intake = hwMap.get(DcMotorEx.class, "intakeRoller");
        left = hwMap.get(CRServo.class, "leftIntake");
        right = hwMap.get(CRServo.class, "rightIntake");

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setDirection(DcMotorEx.Direction.FORWARD);
        //        flywheel.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);
        left.setDirection(CRServo.Direction.REVERSE);
        right.setDirection(CRServo.Direction.FORWARD);
    }
    
    public void off() {

        intake.setPower(0);
        left.setPower(0);
        right.setPower(0);
    }
    public void on() {
        intake.setVelocity(intakeSpeed);
        left.setPower(1.0);
        right.setPower(1.0);
    }

    public double getIntakeSpeed() {return intake.getVelocity();}
    public void updateFlywheelSpeedUp() {
        intakeSpeed = intakeSpeed + 50;
    }
    public void updateFlywheelSpeedDown() {
        intakeSpeed = intakeSpeed - 50;
    }
    public void update() {}
}
