package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp(name="Shooter Test", group="Linear OpMode")
public class ShootertestingteleOP extends LinearOpMode {


    @Override
    public void runOpMode() {

        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                shooter.updateFlywheelSpeedUp();
            }
            if (gamepad1.dpad_down) {
                shooter.updateFlywheelSpeedDown();
            }
            if (gamepad1.a) {
                shooter.shoot();
            } else {
                shooter.off();
            }
            telemetry.addData("shooter's current" , shooter.getFlywheelData());
            telemetry.addData("shooter speed", shooter.getFlywheelSpeed());
            telemetry.update();
        }
    }
}
