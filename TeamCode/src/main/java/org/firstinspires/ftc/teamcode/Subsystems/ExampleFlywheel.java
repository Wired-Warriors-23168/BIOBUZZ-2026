package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;


public class ExampleFlywheel {

    //give the subsystem a set of states it could be in
    public enum State {
        SHOOTING, IDLE, OFF
    }
    private State flywheelState;

    private DcMotorEx flywheel;
    // continuously spinning motors use a PIDF controller, position motors use a PID controller.
    private PIDFController flywheelPIDF;

    //keep all values at the top of the program for easier tuning
    double shootingSpeed = 20;
    double idleSpeed = 5;


    // set up subsystem hardware here
    public ExampleFlywheel() {

        // Declare and set up motor behavior
        flywheel = hardwareMap.get(DcMotorEx.class, "example config name");
        //whatever you named the device in the control hub config

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // this is specific to what you are using the motor for

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // whether it coasts down at zero power or brakes

        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelPIDF = new PIDFController(5, 0, 0, 1);
        // be very careful with PID values, start low and tune up
    }

    //create public methods to set the subsystem into each different state
    public void shoot() {
        flywheelState = State.SHOOTING;
    }
    public void idle() {
        flywheelState = State.IDLE;
    }
    public void off() {
        flywheelState = State.OFF;
    }

    // create a method for each state, this is where all the actual code and logic for the subsystem is written
    private void runShooting() {
        double targetFlywheel = flywheelPIDF.calculate(flywheel.getVelocity(), shootingSpeed);
        flywheel.setPower(targetFlywheel);
    }

    private void runIdle() {
        double targetFlywheel = flywheelPIDF.calculate(flywheel.getVelocity(), idleSpeed);
        flywheel.setPower(targetFlywheel);
    }

    private void runOff() {
        flywheel.setPower(0);
    }

    // an update function that will run continuously, this is where the different state methods are actually ran
    // add all telemetry in this update class as well
    public void update() {

        switch (flywheelState) {
            case SHOOTING:
                runShooting();
                break;
            case IDLE:
                runIdle();
                break;
            case OFF:
                runOff();
                break;
        }

        telemetry.addData("flywheel speed", flywheel.getVelocity());
    }
}
