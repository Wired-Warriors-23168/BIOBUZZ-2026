package org.firstinspires.ftc.teamcode.OpModes.Other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter", group = "Linear Opmode")
public class shooter extends LinearOpMode {

        private DcMotorEx shooterFlywheel = null;

        @Override
        public void runOpMode() {
            shooterFlywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

            shooterFlywheel.setDirection(DcMotor.Direction.REVERSE);

            shooterFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.a) {
                    shooterFlywheel.setPower(0.5);
                } else if (gamepad1.b) {
                    shooterFlywheel.setPower(0.0);
                }

                telemetry.addData("Shooter Power", shooterFlywheel.getPower());
                telemetry.addData("Encoder Ticks", shooterFlywheel.getCurrentPosition());
                telemetry.addData("Velocity", shooterFlywheel.getVelocity());
                telemetry.update();
            }
        }
    }