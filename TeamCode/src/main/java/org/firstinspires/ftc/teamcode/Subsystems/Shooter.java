package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;


public class Shooter {

    private DcMotorEx flywheel;
    double shootingSpeed = 1300;
    double idleSpeed = 1020;


    public Shooter(HardwareMap hwMap) {

        flywheel = hwMap.get(DcMotorEx.class, "flywheel");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
    //        flywheel.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);
    }

    public void shoot() {
        flywheel.setVelocity(shootingSpeed);
    }
    public void idle() {
        flywheel.setVelocity(idleSpeed);
    }
    public void off() {
        flywheel.setPower(0);
    }

    public double getFlywheelSpeed() {return flywheel.getVelocity();}
    public double getFlywheelData() {return  flywheel.getCurrentPosition();}
    public void updateFlywheelSpeedUp() {
        shootingSpeed = shootingSpeed + 50;
    }
    public void updateFlywheelSpeedDown() {
        shootingSpeed = shootingSpeed - 50;
    }
    public void update() {}
    public void flywheelon() {
        flywheel.setVelocity(idleSpeed);
    }
    public void flywheeloff() {
        flywheel.setPower(0);
    }
    public void bigFlywheel() {
        flywheel.setVelocity(shootingSpeed);
    }
    public void bigFlywheelOff() {
        flywheel.setPower(0);
    }
}