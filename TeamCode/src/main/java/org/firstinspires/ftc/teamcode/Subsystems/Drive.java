package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class Drive {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;

    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;
    double maxwheelspeed = 2400;


    public Drive(HardwareMap hwMap) {
    
        frontLeftDrive = hwMap.get(DcMotorEx .class,"front_left_drive");
        backLeftDrive = hwMap.get(DcMotorEx .class,"back_left_drive");
        frontRightDrive = hwMap.get(DcMotorEx .class,"front_right_drive");
        backRightDrive = hwMap.get(DcMotorEx .class,"back_right_drive");

        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.REVERSE);

    }
    
    public void updateDrive(double lefty, double leftx, double rightx) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        frontLeftSpeed = maxwheelspeed * (-lefty + leftx + rightx);
        frontRightSpeed = maxwheelspeed * (-lefty - leftx - rightx);
        backLeftSpeed = maxwheelspeed * (-lefty - leftx + rightx);
        backRightSpeed = maxwheelspeed * (-lefty + leftx - rightx);

//        max = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));
//        max = Math.max(max, Math.abs(backLeftSpeed));
//        max = Math.max(max, Math.abs(backRightSpeed));
//
//        if (max > maxwheelspeed) {
//            frontLeftSpeed /= max;
//            frontRightSpeed /= max;
//            backLeftSpeed /= max;
//            backRightSpeed /= max;
//        }


        frontLeftDrive.setVelocity(frontLeftSpeed);
        frontRightDrive.setVelocity(frontRightSpeed);
        backLeftDrive.setVelocity(backLeftSpeed);
        backRightDrive.setVelocity(backRightSpeed);




        }
        public double[] getVelocities() {
            return new double[] {frontLeftDrive.getVelocity(), frontRightDrive.getVelocity(), backLeftDrive.getVelocity(), backRightDrive.getVelocity()};
        }

        public double[] getTargetVelocities() {
            return new double[] {frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed};
        }
    }