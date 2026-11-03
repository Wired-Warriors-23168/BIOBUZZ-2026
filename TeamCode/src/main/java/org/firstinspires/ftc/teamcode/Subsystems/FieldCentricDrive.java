package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricDrive {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;

    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;
    IMU imu;

    double maxwheelspeed = 2400;


    public FieldCentricDrive(HardwareMap hwMap) {

        frontLeftDrive = hwMap.get(DcMotorEx .class,"front_left_drive");
        backLeftDrive = hwMap.get(DcMotorEx .class,"back_left_drive");
        frontRightDrive = hwMap.get(DcMotorEx .class,"front_right_drive");
        backRightDrive = hwMap.get(DcMotorEx .class,"back_right_drive");

        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.REVERSE);

        // encoder ticks per second, will be very small values
//        frontRightDrive.setVelocityPIDFCoefficients(5, 0, 0, 2);


        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void updateDrive(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        driveRobotCentric(newForward, newRight, rotate);
    }

    public void driveRobotCentric(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxVelocity = 2400.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setVelocity(maxVelocity * (frontLeftPower / maxPower));
        frontRightDrive.setVelocity(maxVelocity * (frontRightPower / maxPower));
        backLeftDrive.setVelocity(maxVelocity * (backLeftPower / maxPower));
        backRightDrive.setVelocity(maxVelocity * (backRightPower / maxPower));
    }
    public double[] getVelocities() {
        return new double[] {frontLeftDrive.getVelocity(), frontRightDrive.getVelocity(), backLeftDrive.getVelocity(), backRightDrive.getVelocity()};
    }

    public double[] getTargetVelocities() {
        return new double[] {frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed};
    }
}