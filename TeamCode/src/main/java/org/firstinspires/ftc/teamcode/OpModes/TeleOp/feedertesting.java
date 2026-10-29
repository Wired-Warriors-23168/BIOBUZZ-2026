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

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger>0.1) {
                feeder.off();
            }
            if (gamepad1.left_trigger<0.1) {
                feeder.on();
            }
            telemetry.update();
        }
    }
}
