package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ExampleFlywheel {

    //your motor
    private DcMotorEx flywheelmotor;

    //keep all values at the top of the program for easier tuning
    double shootingSpeed = 20;
    double idleSpeed = 5;


    // set up subsystem hardware here
    public ExampleFlywheel(HardwareMap hwMap) {

        // Declare and set up motor behavior
        flywheelmotor = hwMap.get(DcMotorEx.class, "example config name");
        //whatever you named the device in the control hub config

        flywheelmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // this is specific to what you are using the motor for

        flywheelmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // whether it coasts down at zero power or brakes

        flywheelmotor.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelmotor.setVelocityPIDFCoefficients(0.002, 0, 0, 0.2);
        // be very careful with PID values, start low and tune up
    }

    //create public methods for all the function you want the subsystems to do, these will be ran in the opmode
    public void shoot() {
        flywheelmotor.setVelocity(shootingSpeed);
    }
    public void idle() {
        flywheelmotor.setVelocity(idleSpeed);
    }
    public void off() {
        flywheelmotor.setPower(0);
    }

    // a if info from this subsystem is needed in other files, write a method like this and call it in those other files
    public double getFlywheelSpeed() {
        return flywheelmotor.getVelocity();
    }

    // an update function that will run continuously, not needed here
    public void update() {

    }
}
