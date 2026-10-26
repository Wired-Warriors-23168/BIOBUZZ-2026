package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp(name="intake Test", group="Linear OpMode")
public class ServoTestingTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() {

        Intake intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intake.on();
            }
            if (gamepad1.b) {
                intake.off();
            }
            telemetry.addData("intake  speed", intake.getFlywheelSpeed());
            telemetry.update();
        }
    }
}
