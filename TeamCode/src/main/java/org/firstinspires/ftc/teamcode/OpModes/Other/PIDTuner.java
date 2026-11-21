package org.firstinspires.ftc.teamcode.OpModes.Other;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.function.Supplier;
import java.util.function.Consumer;

@TeleOp(name = "Motor PIDF Tuner")
public class PIDTuner extends LinearOpMode {

    private static final String MOTOR_NAME = "testMotor";

    private volatile double p, i, d, f;
    private volatile double targetVelocity = 0;

    // New Dashboard variables
    private volatile boolean usePowerControl = false;
    private volatile double power = 0;

    // Supports Double, Boolean, and other types
    private <T> void addSetting(
            FtcDashboard dashboard,
            String name,
            Supplier<T> getter,
            Consumer<T> setter) {

        dashboard.addConfigVariable(
                "LIVE_MOTOR_PIDF",
                name,
                new ValueProvider<T>() {
                    @Override
                    public T get() {
                        return getter.get();
                    }

                    @Override
                    public void set(T value) {
                        setter.accept(value);
                    }
                }
        );
    }

    @Override
    public void runOpMode() {

        DcMotorEx motor = hardwareMap.get(
                DcMotorEx.class, MOTOR_NAME
        );

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(0);

        PIDFCoefficients original = motor.getPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        p = original.p;
        i = original.i;
        d = original.d;
        f = original.f;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // PIDF controls
        addSetting(dashboard, "P", () -> p, v -> p = v);
        addSetting(dashboard, "I", () -> i, v -> i = v);
        addSetting(dashboard, "D", () -> d, v -> d = v);
        addSetting(dashboard, "F", () -> f, v -> f = v);

        addSetting(dashboard, "TargetVelocity",
                () -> targetVelocity,
                v -> targetVelocity = v);

        // Power controls
        addSetting(dashboard, "UsePowerControl",
                () -> usePowerControl,
                v -> usePowerControl = v);

        addSetting(dashboard, "Power",
                () -> power,
                v -> power = v);

        double lastP = p;
        double lastI = i;
        double lastD = d;
        double lastF = f;

        boolean lastPowerMode = false;

        telemetry.addLine("Ready to tune");
        telemetry.update();

        try {
            waitForStart();

            while (opModeIsActive()) {

                boolean powerMode = usePowerControl;
                boolean modeChanged = powerMode != lastPowerMode;

                // Switch motor control modes
                if (modeChanged) {

                    motor.setPower(0);

                    if (powerMode) {
                        motor.setMode(
                                DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        );
                    } else {
                        motor.setMode(
                                DcMotor.RunMode.RUN_USING_ENCODER
                        );
                    }

                    lastPowerMode = powerMode;
                }

                double newP = p;
                double newI = i;
                double newD = d;
                double newF = f;

                if (powerMode) {

                    // Open-loop motor power
                    double appliedPower = Math.max(
                            -1.0, Math.min(1.0, power)
                    );

                    motor.setPower(appliedPower);

                } else {

                    // Update PIDF if gains or mode changed
                    if (modeChanged ||
                            newP != lastP ||
                            newI != lastI ||
                            newD != lastD ||
                            newF != lastF) {

                        motor.setVelocityPIDFCoefficients(
                                newP, newI, newD, newF
                        );

                        lastP = newP;
                        lastI = newI;
                        lastD = newD;
                        lastF = newF;
                    }

                    motor.setVelocity(targetVelocity);
                }

                double actualVelocity = motor.getVelocity();
                double target = targetVelocity;

                TelemetryPacket packet = new TelemetryPacket();

                packet.put("Actual Velocity", actualVelocity);
                packet.put("Target Velocity",
                        powerMode ? 0 : target);
                packet.put("Velocity Error",
                        powerMode ? 0 : target - actualVelocity);
                packet.put("Motor Power", motor.getPower());
                packet.put("Power Mode", powerMode);

                dashboard.sendTelemetryPacket(packet);

                telemetry.addData("Mode",
                        powerMode ? "POWER" : "VELOCITY PIDF");
                telemetry.addData("Actual Velocity",
                        actualVelocity);
                telemetry.addData("Power", motor.getPower());
                telemetry.update();

                sleep(50);
            }

        } finally {
            motor.setPower(0);
        }
    }
}