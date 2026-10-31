package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp(name="FinalTeleOp", group="Linear OpMode")
public class FinalTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() {

        Feeder feeder = new Feeder(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter flywheel = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.left_trigger>0.1) {
                feeder.on();
            }
            if (gamepad2.left_trigger<0.1) {
                feeder.feederOff();
            }
            if (gamepad2.right_trigger>0.1) {
                feeder.gateon();
            }
            if (gamepad2.right_trigger<0.1) {
                feeder.gateoff();
            }
            if (gamepad2.y) {
                flywheel.flywheelon();
            }
            if (gamepad2.a) {
                flywheel.flywheeloff();
                flywheel.bigFlywheelOff();
            }
            if (gamepad2.x) {
                flywheel.bigFlywheel();
            }
            if (gamepad1.right_bumper) {
                intake.on();
            }
            if (gamepad1.left_bumper) {
                intake.off();
            }
            telemetry.addData("flywheel speed", flywheel.getFlywheelSpeed());
            telemetry.addData("feeder speed", feeder.getFeederSpeed());
            telemetry.update();
        }
    }
}
