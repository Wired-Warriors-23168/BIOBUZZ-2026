package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;

@TeleOp(name="Base Drive", group="Linear OpMode")
public class BaseDriveTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() {

        Drive drive = new Drive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            drive.updateDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
            );

            telemetry.addData("front right speed", drive.getVelocities()[1]);
            telemetry.addData("target speed", drive.getTargetVelocities()[1]);
            telemetry.update();
        }
    }
}
