package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Field Centric PIDF Drive", group = "TeleOp")
public class FieldCentric extends LinearOpMode {

    // Dashboard: maximum commanded wheel speed, in encoder ticks/second.
    public static volatile double MAX_SPEED_TICKS_PER_SECOND = 2400;

    // Dashboard: separate built-in velocity PIDF gains for each motor.
    public static class Gains {
        public double P;
        public double I;
        public double D;
        public double F;
    }

    public static Gains FRONT_LEFT = new Gains();
    public static Gains BACK_LEFT = new Gains();
    public static Gains FRONT_RIGHT = new Gains();
    public static Gains BACK_RIGHT = new Gains();

    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    private IMU imu;

    // Remember the last applied values to avoid unnecessary Hub writes.
    private final double[][] appliedGains = new double[4][4];

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
        telemetry.setMsTransmissionInterval(100);

        // Change names to match your robot configuration.
        frontLeft = hardwareMap.get(
                DcMotorEx.class, "front_left_drive");
        backLeft = hardwareMap.get(
                DcMotorEx.class, "back_left_drive");
        frontRight = hardwareMap.get(
                DcMotorEx.class, "front_right_drive");
        backRight = hardwareMap.get(
                DcMotorEx.class, "back_right_drive");

        // Keep your existing directions if they already work.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        DcMotorEx[] motors = {
                frontLeft, backLeft, frontRight, backRight
        };

        for (DcMotorEx motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Load each motor's current velocity gains.
        // These appear in Dashboard after pressing INIT.
        loadGains(frontLeft, FRONT_LEFT, 0);
        loadGains(backLeft, BACK_LEFT, 1);
        loadGains(frontRight, FRONT_RIGHT, 2);
        loadGains(backRight, BACK_RIGHT, 3);

        imu = hardwareMap.get(IMU.class, "imu");

        // CHANGE these to match how your Control Hub is mounted.
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addLine("Ready: face field-forward before START.");
        telemetry.addLine("Dashboard: edit max speed and motor PIDF gains.");
        telemetry.addLine("Y: reset field-forward heading.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        imu.resetYaw();

        boolean previousY = false;

        try {
            while (opModeIsActive()) {

                // Apply gains only when changed in Dashboard.
                applyGains(frontLeft, FRONT_LEFT, 0);
                applyGains(backLeft, BACK_LEFT, 1);
                applyGains(frontRight, FRONT_RIGHT, 2);
                applyGains(backRight, BACK_RIGHT, 3);

                // Reset once per Y press.
                boolean currentY = gamepad1.y;
                if (currentY && !previousY) {
                    imu.resetYaw();
                }
                previousY = currentY;

                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double heading = imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS);

                // Field-relative joystick -> robot-relative movement.
                double robotStrafe =
                        strafe * Math.cos(-heading)
                                - forward * Math.sin(-heading);

                double robotForward =
                        strafe * Math.sin(-heading)
                                + forward * Math.cos(-heading);

                double fl = robotForward + robotStrafe + turn;
                double bl = robotForward - robotStrafe + turn;
                double fr = robotForward - robotStrafe - turn;
                double br = robotForward + robotStrafe - turn;

                // Preserve wheel ratios while limiting magnitude to 1.
                double denominator = Math.max(
                        1.0,
                        Math.max(
                                Math.max(Math.abs(fl), Math.abs(bl)),
                                Math.max(Math.abs(fr), Math.abs(br))
                        )
                );

                double maxSpeed = MAX_SPEED_TICKS_PER_SECOND;

                if (!Double.isFinite(maxSpeed) || maxSpeed < 0) {
                    maxSpeed = 0;
                }

                double flTarget = fl / denominator * maxSpeed;
                double blTarget = bl / denominator * maxSpeed;
                double frTarget = fr / denominator * maxSpeed;
                double brTarget = br / denominator * maxSpeed;

                // REV Hub performs the velocity PIDF control.
                frontLeft.setVelocity(flTarget);
                backLeft.setVelocity(blTarget);
                frontRight.setVelocity(frTarget);
                backRight.setVelocity(brTarget);

                // Numeric fields can be graphed in FTC Dashboard.
                telemetry.addData("HeadingDeg", Math.toDegrees(heading));
                telemetry.addData("MaxSpeed", maxSpeed);

                telemetry.addData("FL_Target", flTarget);
                telemetry.addData("FL_Actual", frontLeft.getVelocity());

                telemetry.addData("BL_Target", blTarget);
                telemetry.addData("BL_Actual", backLeft.getVelocity());

                telemetry.addData("FR_Target", frTarget);
                telemetry.addData("FR_Actual", frontRight.getVelocity());

                telemetry.addData("BR_Target", brTarget);
                telemetry.addData("BR_Actual", backRight.getVelocity());

                telemetry.update();
                idle();
            }
        } finally {
            for (DcMotorEx motor : motors) {
                motor.setVelocity(0);
                motor.setPower(0);
            }
        }
    }

    private void loadGains(DcMotorEx motor, Gains gains, int index) {
        PIDFCoefficients current = motor.getPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        gains.P = current.p;
        gains.I = current.i;
        gains.D = current.d;
        gains.F = current.f;

        rememberGains(index, current.p, current.i, current.d, current.f);
    }

    private void applyGains(DcMotorEx motor, Gains gains, int index) {
        double p = gains.P;
        double i = gains.I;
        double d = gains.D;
        double f = gains.F;

        // Ignore invalid Dashboard entries.
        if (!Double.isFinite(p) || !Double.isFinite(i)
                || !Double.isFinite(d) || !Double.isFinite(f)
                || p < 0 || i < 0 || d < 0 || f < 0) {
            return;
        }

        double[] last = appliedGains[index];

        if (p != last[0] || i != last[1]
                || d != last[2] || f != last[3]) {

            motor.setVelocityPIDFCoefficients(p, i, d, f);
            rememberGains(index, p, i, d, f);
        }
    }

    private void rememberGains(
            int index, double p, double i, double d, double f) {

        appliedGains[index][0] = p;
        appliedGains[index][1] = i;
        appliedGains[index][2] = d;
        appliedGains[index][3] = f;
    }
}