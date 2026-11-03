package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp(name="FinalTeleOp", group="Linear OpMode")
public class FinalTeleOp extends LinearOpMode {

    boolean isRamen = true;

    @Override
    public void runOpMode() {

        Feeder feeder = new Feeder(hardwareMap, isRamen);
        Intake intake = new Intake(hardwareMap, isRamen);
        Shooter flywheel = new Shooter(hardwareMap, isRamen);
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap, isRamen);
        Hood hood = new Hood(hardwareMap, isRamen);
        waitForStart();

        while (opModeIsActive()) {

            drive.updateDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
            if (gamepad1.yWasPressed()) {
                drive.resetYaw();
            }
            if (gamepad2.left_trigger>0.1) {
                feeder.on();
                intake.on();
                feeder.reverse();
            }
            if (gamepad2.left_trigger<0.1) {
                feeder.feederOff();
                intake.off();
                feeder.gateoff();
            }
            if (gamepad2.right_trigger>0.1) {
                feeder.gateon();
                feeder.on();
            }
            if (gamepad2.right_trigger<0.1) {
                feeder.gateoff();
                feeder.feederOff();
            }
            if (gamepad2.dpad_left) {
                hood.pollenPos();
            }
            if (gamepad2.dpad_right) {
                hood.nectarPos();
            }
            if (gamepad2.y) {
                flywheel.flywheelon();
            }
            if (gamepad2.a) {
                flywheel.flywheeloff();
            }
            if (gamepad2.x) {
                flywheel.bigFlywheel();
            }
            telemetry.addData("flywheel speed", flywheel.getFlywheelSpeed());
            telemetry.addData("feeder speed", feeder.getFeederSpeed());
            telemetry.update();
        }
    }
}
