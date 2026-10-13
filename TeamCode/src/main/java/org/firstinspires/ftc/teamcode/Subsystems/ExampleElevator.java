package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class ExampleElevator {

        //give the subsystem a set of states it could be in
        public enum State {
            UP, STORED
        }
        private State elevatorState;

        private DcMotorEx elevator;
        // continuously spinning motors use a PIDF controller, position motors use a PID controller.
        private PIDFController elevatorPID;

        //keep all values at the top of the program for easier tuning
        double TopPosition = 5;
        double storedPosition = 0;


        // set up subsystem hardware here
        public ExampleElevator(HardwareMap hwMap) {

            // Declare and set up motor behavior
            elevator = hwMap.get(DcMotorEx.class, "example config name");
            //whatever you named the device in the control hub config

            elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // this is specific to what you are using the motor for

            elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // whether it coasts down at zero power or brakes

            elevator.setDirection(DcMotorEx.Direction.FORWARD);
            elevatorPID = new PIDController(1, 0, 0);
            // be very careful with PID values, start low and tune up
        }

        //create public methods to set the subsystem into each different state
        public void store() {
            elevatorState = State.STORED;
        }
        public void up() {
            elevatorState = State.UP;
        }


        // create a method for each state, this is where all the actual code and logic for the subsystem is written
        private void runStored() {
            double targetElevator = elevatorPID.calculate(elevator.getCurrentPosition(), storedPosition);
            elevator.setPower(targetElevator);
        }

        private void runUp() {
            double targetElevator = elevatorPID.calculate(elevator.getCurrentPosition(), TopPosition);
            elevator.setPower(targetElevator);
        }



        // an update function that will run continuously, this is where the different state methods are actually ran
        // add all telemetry in this update class as well
        public void update() {

            switch (elevatorState) {
                case STORED:
                    runStored();
                    break;
                case UP:
                    runUp();
                    break;
            }

            telemetry.addData("flywheel speed", elevator.getCurrentPosition());
        }
    }
