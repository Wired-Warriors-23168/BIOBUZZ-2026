package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
