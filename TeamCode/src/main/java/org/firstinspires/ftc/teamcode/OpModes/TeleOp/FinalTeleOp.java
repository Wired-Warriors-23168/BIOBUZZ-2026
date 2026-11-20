package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Ramp;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

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
        Ramp ramp = new Ramp(hardwareMap, isRamen);
        //Vision vision = new Vision(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
//            vision.update();
//            telemetry.addData("connected", vision.limelight.isConnected());
//            telemetry.addData("time since lst update", vision.limelight.getTimeSinceLastUpdate());
//            telemetry.addData("tag IDs", vision.tagIDs());
//            telemetry.addData("tx", vision.getTx());
//            telemetry.addData("valid", vision.limelight.getLatestResult().isValid());
//            telemetry.addData("pipeline", vision.limelight.getStatus().getPipelineIndex());
            drive.driveRobotCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
            if (gamepad1.y) {
                ramp.setRampPosUp();
            }
            if (gamepad1.a) {
                ramp.setRampPosDown();
            }
            if (gamepad1.b) {

            }
            if (gamepad1.x) {

            }
            if (gamepad1.left_bumper) {
                drive.autoAim();
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
            if (gamepad2.left_bumper) {
                intake.reverseintake();
            }
            if (gamepad2.y) {
                flywheel.shootPollen();
                hood.pollenPos();
            }
            if (gamepad2.x) {
                flywheel.shootNectar();
                hood.nectarPos();
            }
            if (gamepad1.dpadUpWasPressed()) {
                hood.nectarPos += 0.02;
                hood.nectarPos();
            }
            if (gamepad1.dpadDownWasPressed()) {
                hood.nectarPos -= 0.02;
                hood.nectarPos();

            }
            if (gamepad1.rightBumperWasPressed()) {
                flywheel.updateFlywheelSpeedUp();
            }
            if (gamepad1.leftBumperWasPressed()) {
                flywheel.updateFlywheelSpeedDown();
            }
            telemetry.addData("flywheel speed", flywheel.getFlywheelSpeed());
            telemetry.addData("hoodpos", "%.2f", hood.getHoodPos());
            telemetry.addData("target hood", "%.2f", hood.nectarPos);

            telemetry.addData("feeder speed", feeder.getFeederSpeed());
            telemetry.update();
        }
    }
}
