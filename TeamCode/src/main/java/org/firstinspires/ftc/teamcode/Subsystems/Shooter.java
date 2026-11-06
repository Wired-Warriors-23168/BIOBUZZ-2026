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
    double nectarSpeed = 0;
    double pollenSpeed = 0;
    double idleSpeed = 0;


    public Shooter(HardwareMap hwMap, boolean isRamen) {
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");

        if (isRamen) {
            flywheel.setDirection(DcMotorEx.Direction.REVERSE);
            nectarSpeed = 1260;
            pollenSpeed = 1060;
        } else {
            flywheel.setDirection(DcMotorEx.Direction.REVERSE);
            nectarSpeed = 1400;
            pollenSpeed = 1120;
        }

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setVelocityPIDFCoefficients(100, 0, 0, 17.8);
    }

       public Shooter(HardwareMap hwMap) {
           flywheel = hwMap.get(DcMotorEx.class, "flywheel");
            flywheel.setDirection(DcMotorEx.Direction.REVERSE);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setVelocityPIDFCoefficients(100, 0, 0, 17.8);
        }

    public void shootNectar() {
        flywheel.setVelocity(nectarSpeed);
    }
    public void shootPollen() {
        flywheel.setVelocity(pollenSpeed);
    }
    public void shoot() {
        flywheel.setVelocity(nectarSpeed);
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
        nectarSpeed += 5;
        flywheel.setVelocity(nectarSpeed);
    }
    public void updateFlywheelSpeedDown() {
        nectarSpeed -= 5;
        flywheel.setVelocity(nectarSpeed);
    }
    public void update() {}
    public void flywheelon() {
        flywheel.setVelocity(idleSpeed);
    }
    public void flywheeloff() {
        flywheel.setPower(0);
    }
    public void bigFlywheel() {
        flywheel.setVelocity(nectarSpeed);
    }
}