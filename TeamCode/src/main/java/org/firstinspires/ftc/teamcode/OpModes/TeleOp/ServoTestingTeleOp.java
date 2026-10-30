package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name="intake Test", group="Linear OpMode")
public class ServoTestingTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() {

        Intake intake = new Intake(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intake.on();
            }
            if (gamepad1.b) {
                intake.off();
            }
            telemetry.addData("intake  speed", intake.getIntakeSpeed());
            telemetry.update();
        }
    }
}
