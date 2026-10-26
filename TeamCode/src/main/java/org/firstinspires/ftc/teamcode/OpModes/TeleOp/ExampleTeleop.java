package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ExampleElevator;
import org.firstinspires.ftc.teamcode.Subsystems.ExampleFlywheel;


//give the opmode a name and make sure it is not @Disabled, it stops the OPmode from being sent to the control hub
@TeleOp(name="Example Teleop", group="Linear OpMode")
@Disabled
public class ExampleTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {

        //declare all of your subsystems
        ExampleFlywheel flywheel = new ExampleFlywheel(hardwareMap);
        ExampleElevator elevator = new ExampleElevator(hardwareMap);


        // this loop will run continuously while the robot is enabled
        waitForStart();
        while (opModeIsActive()) {

            // set up buttons to activate certain subsystem functions
            if (gamepad1.a) {
                flywheel.shoot();

            } else if (gamepad1.x) {
                flywheel.idle();

            } else if (gamepad1.b) {
                flywheel.off();
            }

            if (gamepad1.right_bumper) {
                elevator.up();
            } else if (gamepad1.left_bumper) {
                elevator.store();
            }

            // make sure your subsystem is updating every loop
            flywheel.update();
            elevator.update();
        }
    }
}
