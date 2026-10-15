package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class ExampleElevator {

        // your motor
        private DcMotorEx elevator;

        //keep all values at the top of the program for easier tuning
        int topPosition = 5;
        int storedPosition = 0;


        // set up subsystem hardware here
        public ExampleElevator(HardwareMap hwMap) {

            // Declare and set up motor behavior
            elevator = hwMap.get(DcMotorEx.class, "example config name");
            //whatever you named the device in the control hub config

            //use RUN_TO_POSITION for subsystems like elevators
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //how close the elevator needs to be before it stops moving
            elevator.setTargetPositionTolerance(5);


            elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // whether it coasts down at zero power or brakes

            elevator.setDirection(DcMotorEx.Direction.FORWARD);
            elevator.setPositionPIDFCoefficients(0.05);
            // be very careful with PID values, start low and tune up
        }

        public void up() {
            elevator.setTargetPosition(topPosition);
        }
        public void store() {
            elevator.setTargetPosition(storedPosition);
        }

        public int getElevatorPosition() {
            return elevator.getCurrentPosition();
        }

        // an update function that will run continuously
        public void update() {


        }
    }
