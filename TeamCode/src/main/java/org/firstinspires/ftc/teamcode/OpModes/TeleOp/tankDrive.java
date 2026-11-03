package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="tank drive", group="Linear OpMode")
public class tankDrive extends LinearOpMode {

    //tank drive

    //have 4 total motors
    //setPower on 2 to the left stick
    //setPower on the other 2 to right stick
    // put these joystick values on telemetry
    private DcMotorEx leftmotor = null;
    private DcMotorEx rightmotor = null;
    private DcMotorEx bottemrightmotor = null;
    private DcMotorEx bottemleftmotor = null;

    double variable = 0;
    private float leftpower = 0;
    private float rightpower = 0;


    @Override
    public void runOpMode() {
        bottemrightmotor  = hardwareMap.get(DcMotorEx.class, "bottemrightmotor");
        leftmotor  = hardwareMap.get(DcMotorEx.class, "leftmotor");
        rightmotor  = hardwareMap.get(DcMotorEx.class, "rightmotor");
        bottemleftmotor  = hardwareMap.get(DcMotorEx.class, "bottemleftmotor");
        // run until the end of the match (driver presses STOP)
        telemetry.addData("left_stick_y", leftpower);
        telemetry.addData("right_stick_y", rightpower);

        leftmotor.setDirection(DcMotor.Direction.FORWARD);
        rightmotor.setDirection(DcMotor.Direction.FORWARD);

        leftpower = gamepad1.right_stick_y;
        leftmotor.setPower(leftpower);
        //leftpower = gamepad1.right_stick_y;
        bottemleftmotor.setPower(leftpower);

        rightpower = gamepad1.right_stick_y;
        rightmotor.setPower(rightpower);
//        rightpower = gamepad1.right_stick_y;
        bottemrightmotor.setPower(rightpower);


        }
    }

