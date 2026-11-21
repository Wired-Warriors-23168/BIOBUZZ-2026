package org.firstinspires.ftc.teamcode.OpModes.Other;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.function.Supplier;
import java.util.function.Consumer;

@TeleOp(name = "Mecanum PID Tuner")
public class MecanumPIDTuner extends LinearOpMode {

    // EDIT THESE TO MATCH YOUR ROBOT CONFIGURATION
    private static final String[] MOTOR_NAMES = {
            "frontLeft", "frontRight", "backLeft", "backRight"
    };

    private static final String[] LABELS = {
            "FL", "FR", "BL", "BR"
    };

    // Order: FL, FR, BL, BR
    // All patterns assume positive velocity = forward
    private static final double[][] PATTERNS = {
            { 1,  0,  0,  0}, // 0: Front Left only
            { 0,  1,  0,  0}, // 1: Front Right only
            { 0,  0,  1,  0}, // 2: Back Left only
            { 0,  0,  0,  1}, // 3: Back Right only
            { 1,  1,  1,  1}, // 4: Forward
            { 1, -1, -1,  1}, // 5: Strafe right
            { 1, -1,  1, -1}  // 6: Rotate
    };

    private final DcMotorEx[] motors = new DcMotorEx[4];

    // Dashboard values
    private volatile double p, i, d, f;
    private volatile double targetVelocity = 0;
    private volatile double power = 0;

    private volatile boolean usePowerControl = false;
    private volatile boolean enableMotors = false;

    // 0-6, as defined in PATTERNS
    private volatile int testPattern = 0;

    // Direct Dashboard registration
    // This bypasses the @Config registration problem.
    private <T> void addSetting(
            FtcDashboard dashboard,
            String name,
            Supplier<T> getter,
            Consumer<T> setter) {

        dashboard.addConfigVariable(
                "MECANUM_PIDF_TUNER",
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

    private void registerDashboard(FtcDashboard dashboard) {

        addSetting(dashboard, "P",
                () -> p, v -> p = v);
        addSetting(dashboard, "I",
                () -> i, v -> i = v);
        addSetting(dashboard, "D",
                () -> d, v -> d = v);
        addSetting(dashboard, "F",
                () -> f, v -> f = v);

        addSetting(dashboard, "TargetVelocity",
                () -> targetVelocity,
                v -> targetVelocity = v);

        addSetting(dashboard, "UsePowerControl",
                () -> usePowerControl,
                v -> usePowerControl = v);

        addSetting(dashboard, "Power",
                () -> power,
                v -> power = v);

        addSetting(dashboard, "TestPattern",
                () -> testPattern,
                v -> testPattern = v);

        addSetting(dashboard, "EnableMotors",
                () -> enableMotors,
                v -> enableMotors = v);
    }

    private void stopMotors() {
        for (DcMotorEx motor : motors) {
            if (motor != null) {
                motor.setPower(0);
            }
        }
    }

    @Override
    public void runOpMode() {

        // Map all four motors
        for (int n = 0; n < 4; n++) {
            motors[n] = hardwareMap.get(
                    DcMotorEx.class, MOTOR_NAMES[n]
            );
        }

        // EDIT DIRECTIONS TO MATCH YOUR WORKING TELEOP.
        motors[0].setDirection(
                DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(
                DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(
                DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(
                DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );
            motor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );
        }

        // Use current FL motor PIDF as starting values.
        PIDFCoefficients original =
                motors[0].getPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER
                );

        p = original.p;
        i = original.i;
        d = original.d;
        f = original.f;

        FtcDashboard dashboard =
                FtcDashboard.getInstance();

        registerDashboard(dashboard);

        // Ensure gains get applied on the first loop.
        double lastP = Double.NaN;
        double lastI = Double.NaN;
        double lastD = Double.NaN;
        double lastF = Double.NaN;

        boolean lastPowerMode = false;
        int lastPattern = testPattern;

        telemetry.addLine("Mecanum PIDF tuner ready");
        telemetry.addLine("Motors disabled by default");
        telemetry.update();

        try {
            waitForStart();

            while (opModeIsActive()) {

                boolean powerMode = usePowerControl;

                int pattern = Math.max(
                        0, Math.min(6, testPattern)
                );

                // Disable motion when switching test
                // patterns or controller modes.
                if (pattern != lastPattern ||
                        powerMode != lastPowerMode) {

                    enableMotors = false;
                    stopMotors();

                    if (powerMode != lastPowerMode) {
                        DcMotor.RunMode newMode =
                                powerMode
                                        ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                                        : DcMotor.RunMode.RUN_USING_ENCODER;

                        for (DcMotorEx motor : motors) {
                            motor.setMode(newMode);
                        }
                    }

                    lastPattern = pattern;
                    lastPowerMode = powerMode;
                }

                // Read current gains
                double newP = p;
                double newI = i;
                double newD = d;
                double newF = f;

                // Update all four velocity controllers
                // when a PIDF value changes.
                if (!powerMode &&
                        (newP != lastP ||
                                newI != lastI ||
                                newD != lastD ||
                                newF != lastF)) {

                    for (DcMotorEx motor : motors) {
                        motor.setVelocityPIDFCoefficients(
                                newP, newI, newD, newF
                        );
                    }

                    lastP = newP;
                    lastI = newI;
                    lastD = newD;
                    lastF = newF;
                }

                boolean enabled = enableMotors;

                double command = 0;

                if (enabled) {
                    command = powerMode
                            ? Math.max(-1, Math.min(1, power))
                            : targetVelocity;
                }

                TelemetryPacket packet =
                        new TelemetryPacket();

                packet.put("Enabled", enabled);
                packet.put("Power Mode", powerMode);
                packet.put("Test Pattern", pattern);

                double totalAbsoluteError = 0;

                for (int n = 0; n < 4; n++) {

                    double wheelCommand =
                            command * PATTERNS[pattern][n];

                    if (powerMode) {
                        motors[n].setPower(wheelCommand);
                    } else {
                        motors[n].setVelocity(wheelCommand);
                    }

                    double actual =
                            motors[n].getVelocity();

                    packet.put(
                            LABELS[n] + " Actual",
                            actual
                    );

                    if (powerMode) {
                        packet.put(
                                LABELS[n] + " Power",
                                wheelCommand
                        );
                    } else {
                        packet.put(
                                LABELS[n] + " Target",
                                wheelCommand
                        );

                        double error =
                                wheelCommand - actual;

                        packet.put(
                                LABELS[n] + " Error",
                                error
                        );

                        totalAbsoluteError +=
                                Math.abs(error);
                    }

                    telemetry.addData(
                            LABELS[n] + " Velocity",
                            "%.1f", actual
                    );
                }

                if (!powerMode) {
                    packet.put(
                            "Total Absolute Error",
                            totalAbsoluteError
                    );
                }

                dashboard.sendTelemetryPacket(packet);

                telemetry.addData(
                        "Mode",
                        powerMode ? "POWER" : "VELOCITY PIDF"
                );
                telemetry.addData("Pattern", pattern);
                telemetry.addData("Enabled", enabled);
                telemetry.update();

                sleep(50);
            }
        } finally {
            enableMotors = false;
            stopMotors();
        }
    }
}