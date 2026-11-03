package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name="hoodtest", group="Linear OpMode")
public class hoodTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        Hood hood = new Hood(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                hood.pollenPos();
            }
            if (gamepad1.dpad_right) {
                hood.nectarPos();
            }
            telemetry.addData("hoodPos", hood.getHoodPos());
            telemetry.update();
        }
    }
}
