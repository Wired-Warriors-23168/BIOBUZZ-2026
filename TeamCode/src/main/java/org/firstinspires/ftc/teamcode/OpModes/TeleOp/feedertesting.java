package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name="feeder test", group="Linear OpMode")
public class feedertesting extends LinearOpMode {


    @Override
    public void runOpMode() {

        Feeder feeder = new Feeder(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger>0.1) {
                feeder.feederOff();
            }
            if (gamepad1.left_trigger<0.1) {
                feeder.on();
            }
            if (gamepad1.right_trigger>0.1) {
                feeder.gateon();
            }
            if (gamepad1.right_trigger<0.1) {
                feeder.gateoff();
            }
            if (gamepad1.right_bumper) {
                intake.on();
            }
            if (gamepad1.left_bumper) {
                intake.off();
            }

            telemetry.addData("feeder speed", feeder.getFeederSpeed());
            telemetry.update();
        }
    }
}
