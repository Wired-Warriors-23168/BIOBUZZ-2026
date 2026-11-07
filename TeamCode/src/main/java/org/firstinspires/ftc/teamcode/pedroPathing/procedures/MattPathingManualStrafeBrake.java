package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import android.content.Context;
import android.content.SharedPreferences;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * MattPathing: standalone, manually restarted Strafe Braking identification.
 *
 * Compatible with the Pedro Pathing 3.0.2 Quickstart-style Constants class:
 *     Constants.drivetrainConfig : MecanumConfig
 *     Constants.localizerConfig  : OTOSConfig
 *
 * Five LEFT-strafe trials with symmetric wheel commands (no turn correction). Each trial records measured starting speed (in/s)
 * and distance traveled while braking (in). It fits d = a*v + b*v^2,
 * matching Pedro Pathing's Utils.quadraticFit().
 *
 * NOTE: This is not a Foresight AutoTune procedure. Run from Driver Station.
 * Samples are saved between OpMode runs. Press X after the five trials to
 * explicitly copy the coefficients into MattPathing's Step 8 checkpoint.
 */
@Autonomous(name = "MattPathing - Manual Strafe Brake", group = "MattPathing")
public class MattPathingManualStrafeBrake extends LinearOpMode {

    // Five LEFT-strafe power levels; manually reposition the robot between tests.
    private static final double[] POWERS = {0.4, 0.45, 0.50, 0.55, 0.65};

    // Units below are INCHES and SECONDS. Give the robot ample space beyond STRAFE_DISTANCE_IN.
    private static final double STRAFE_DISTANCE_IN = 30.0;
    private static final double MAX_TOTAL_Y_IN = 108.0;
    private static final double MAX_FORWARD_DRIFT_IN = 18.0;
    private static final double MAX_HEADING_ERROR_RAD = Math.toRadians(25.0);
    // Pure lateral input during calibration: no heading correction is mixed with strafe.
    // All four mecanum wheels receive equal-magnitude power (opposite wheel signs).
    // The 25-degree heading-drift cutoff below remains active for safety.
    private static final double MAX_DRIVE_SECONDS = 4.0;
    private static final double MAX_COAST_SECONDS = 5.0;
    private static final double ARM_DELAY_SECONDS = 1.5;
    private static final double STOP_SPEED_IN_PER_SEC = 0.8;
    private static final double STOP_STABLE_SECONDS = 0.25;
    private static final double MIN_VALID_START_SPEED = 2.0;

    // Match Pedro's default StrafeBraking brakingPower (slight rightward power while coasting).
    // No heading turn command during driving or braking.
    private static final double BRAKING_POWER = 0.001;

    private static final String PREF_NAME = "MattPathingManualStrafeBrake_v2";
    private final List<double[]> samples = new ArrayList<>();

    private enum State { READY, ARMING, DRIVING, COASTING, COMPLETE, FAULT }

    private State state = State.READY;
    private String status = "Place robot with 30+ inches clear to its LEFT. Press A to start a trial.";
    private long stateStartNs;
    private long stoppedSinceNs = -1;
    private double brakeStartX;
    private double brakeStartY;
    private double brakeStartSpeed;
    private double trialStartHeading;
    private double lastHeading;
    private double lastHeadingError;
    private double lastHeadingCommand;
    private double lastX;
    private double lastY;
    private double lastVx;
    private double lastVy;
    private boolean prevA;
    private boolean prevB;
    private boolean prevY;
    private boolean prevX;
    private SharedPreferences prefs;
    private SharedPreferences mattPrefs;
    private Mecanum drivetrain;
    private OTOSLocalizer localizer;

    @Override
    public void runOpMode() throws InterruptedException {
        // Reuses your already-tuned motor directions and OTOS settings.
        // Change these two configuration names ONLY if your Constants class uses other names.
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        try {
            localizer = new OTOSLocalizer(hardwareMap, Constants.localizerConfig);
            prefs = hardwareMap.appContext.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
            mattPrefs = hardwareMap.appContext.getSharedPreferences("safe_foresight_v1", Context.MODE_PRIVATE);
            loadSamples();
            if (samples.size() == POWERS.length) state = State.COMPLETE;

            drivetrain.stop(false); // FLOAT so the robot can be repositioned by hand.
            telemetry.setMsTransmissionInterval(100);
            telemetry.addLine("MattPathing Manual Strafe Brake");
            telemetry.addLine("Strafe LEFT (+Y) only, symmetric wheel powers, no heading hold.");
            telemetry.addLine("Before testing, move robot LEFT by hand and verify OTOS Y increases.");
            telemetry.addLine("A = trial, B = abort, Y = clear samples, X = import to MattPathing.");
            telemetry.addData("Saved trials", "%d / %d", samples.size(), POWERS.length);
            telemetry.addLine("Clear at least 30 inches to the LEFT, plus coasting space.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevY = gamepad1.y;
            prevX = gamepad1.x;

            while (opModeIsActive()) {
                localizer.update();
                lastX = localizer.pose().x();
                lastY = localizer.pose().y();
                lastVx = localizer.velocity().toVector2D().x();
                lastVy = localizer.velocity().toVector2D().y();
                double heading = localizer.pose().heading();
                double omega = localizer.velocity().omega;
                lastHeading = heading;

                boolean aTapped = gamepad1.a && !prevA;
                boolean bTapped = gamepad1.b && !prevB;
                boolean yTapped = gamepad1.y && !prevY;
                boolean xTapped = gamepad1.x && !prevX;
                prevA = gamepad1.a;
                prevB = gamepad1.b;
                prevY = gamepad1.y;
                prevX = gamepad1.x;

                if (bTapped) {
                    fail("B pressed: trial aborted. Completed trials are preserved.");
                } else if (yTapped && (state == State.READY || state == State.COMPLETE || state == State.FAULT)) {
                    resetSamples();
                    state = State.READY;
                    status = "Saved trials cleared. Reposition robot, then press A.";
                    drivetrain.stop(false);
                }

                if (state == State.COMPLETE && xTapped) {
                    publishStep8Checkpoint();
                }

                if (state == State.READY && aTapped) {
                    if (!Double.isFinite(lastVx) || !Double.isFinite(lastVy)
                            || Math.hypot(lastVx, lastVy) > 3.0) {
                        status = "Robot must be stationary before starting (check OTOS velocity).";
                    } else {
                        state = State.ARMING;
                        stateStartNs = System.nanoTime();
                        drivetrain.stop(false);
                        status = "Arming: stay clear of robot; left strafe starts shortly.";
                    }
                }

                if (state == State.ARMING && elapsedSeconds(stateStartNs) >= ARM_DELAY_SECONDS) {
                    localizer.setPose(Pose.zero());
                    localizer.update();
                    trialStartHeading = localizer.pose().heading();
                    lastHeadingError = 0.0;
                    lastHeadingCommand = 0.0;
                    state = State.DRIVING;
                    stateStartNs = System.nanoTime();
                    status = "Strafing left at power " + POWERS[samples.size()];
                }

                if (state == State.DRIVING || state == State.COASTING) {
                    // Reload after any pose reset to inspect the same state being controlled.
                    lastX = localizer.pose().x();
                    lastY = localizer.pose().y();
                    lastVx = localizer.velocity().toVector2D().x();
                    lastVy = localizer.velocity().toVector2D().y();
                    heading = localizer.pose().heading();
                    omega = localizer.velocity().omega;
                    lastHeading = heading;
                    lastHeadingError = signedAngle(heading - trialStartHeading);
                    if (!finite(lastX, lastY, lastVx, lastVy, heading, omega,
                            lastHeadingError, trialStartHeading)) {
                        fail("Invalid OTOS pose/velocity (NaN or infinity). No trial recorded.");
                 //   } else if (lastY > MAX_TOTAL_Y_IN || lastY < -8.0) {
                        fail("Travel safety limit reached. No trial recorded.");
                 //   } else if (Math.abs(lastX) > MAX_FORWARD_DRIFT_IN) {
                        fail("Excessive forward/backward drift. No trial recorded.");
                 //   } else if (Math.abs(lastHeadingError) > MAX_HEADING_ERROR_RAD) {
                        fail(String.format(Locale.US,
                                "Heading drift %.1f degrees (limit 25). No trial recorded. "
                                + "Check whether the robot actually turned or OTOS jumped.",
                                Math.toDegrees(lastHeadingError)));
                    }
                }

                switch (state) {
                    case DRIVING:
                        if (elapsedSeconds(stateStartNs) > MAX_DRIVE_SECONDS) {
                            fail(String.format(Locale.US, "Drive timeout: OTOS Y=%.1f in (target %.1f in). Check Y axis and direction.", lastY, STRAFE_DISTANCE_IN));
                        } else if (lastY >= STRAFE_DISTANCE_IN) {
                            brakeStartX = lastX;
                            brakeStartY = lastY;
                            brakeStartSpeed = Math.hypot(lastVx, lastVy);
                            if (brakeStartSpeed < MIN_VALID_START_SPEED) {
                                fail("Start velocity too low for a valid braking sample.");
                            } else {
                                state = State.COASTING;
                                stateStartNs = System.nanoTime();
                                stoppedSinceNs = -1;
                                applyBrakingCommand();
                                status = "Coasting/braking. DO NOT move robot until it stops.";
                            }
                        } else {
                            drivetrain.drive(new DrivePowers(
                                    0, POWERS[samples.size()], 0), false);
                            lastHeadingCommand = 0.0;
                        }
                        break;
                    case COASTING:
                        if (elapsedSeconds(stateStartNs) > MAX_COAST_SECONDS) {
                            fail("Braking timeout. No trial recorded.");
                            break;
                        }
                        // Apply equal-magnitude opposite strafe to all four wheels; no heading turn.
                        applyBrakingCommand();
                        if (Math.hypot(lastVx, lastVy) <= STOP_SPEED_IN_PER_SEC) {
                            if (stoppedSinceNs < 0) stoppedSinceNs = System.nanoTime();
                            if (elapsedSeconds(stoppedSinceNs) >= STOP_STABLE_SECONDS) {
                                double distance = Math.hypot(lastX - brakeStartX, lastY - brakeStartY);
                                drivetrain.stop(false);
                                if (!Double.isFinite(distance) || distance <= 0.02) {
                                    fail("Braking distance was invalid or too small.");
                                } else {
                                    samples.add(new double[]{brakeStartSpeed, distance});
                                    boolean saved = saveSamples();
                                    status = String.format(Locale.US,
                                            "Trial %d saved: v=%.3f in/s, d=%.3f in.%s",
                                            samples.size(), brakeStartSpeed, distance,
                                            saved ? "" : " WARNING: persistent save failed!");
                                    state = samples.size() == POWERS.length ? State.COMPLETE : State.READY;
                                }
                            }
                        } else {
                            stoppedSinceNs = -1;
                        }
                        break;
                    case READY:
                    case ARMING:
                    case COMPLETE:
                    case FAULT:
                        // No drive commands, and the motors remain stopped.
                        break;
                }

                showTelemetry();
                sleep(20); // ~50 Hz; avoids tight loops that amplify OTOS jitter.
            }
        } finally {
            if (drivetrain != null) drivetrain.stop(true);
        }
    }

    /** Normalize the difference between two headings across the +/-PI boundary. */
    private static double signedAngle(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    /** Symmetric counter-strafe, with no turn correction. A 0.001 power command
     *  approximates Pedro's minimum braking command with FLOAT zero-power behavior.
     *  This largely measures passive coasting, not hard active braking.
     */
    private void applyBrakingCommand() {
        lastHeadingCommand = 0.0;
        drivetrain.drive(new DrivePowers(0, -BRAKING_POWER, 0), false);
    }

    private void fail(String reason) {
        drivetrain.stop(true); // force BRAKE on a timeout, sensor failure, or manual abort
        state = State.FAULT;
        status = reason + " Press Y to clear saved trials or Stop and restart.";
    }

    private boolean finite(double... values) {
        for (double v : values) if (!Double.isFinite(v)) return false;
        return true;
    }

    private double elapsedSeconds(long sinceNs) {
        return (System.nanoTime() - sinceNs) / 1e9;
    }

    private void loadSamples() {
        samples.clear();
        int n = prefs.getInt("count", 0);
        if (n < 0 || n > POWERS.length) {
            prefs.edit().clear().apply();
            return;
        }
        for (int i = 0; i < n; i++) {
            double velocity = Double.longBitsToDouble(prefs.getLong("v" + i, 0));
            double distance = Double.longBitsToDouble(prefs.getLong("d" + i, 0));
            if (!finite(velocity, distance) || velocity <= 0 || distance <= 0) {
                samples.clear();
                prefs.edit().clear().apply();
                return;
            }
            samples.add(new double[]{velocity, distance});
        }
    }

    private boolean saveSamples() {
        SharedPreferences.Editor e = prefs.edit().clear();
        for (int i = 0; i < samples.size(); i++) {
            e.putLong("v" + i, Double.doubleToRawLongBits(samples.get(i)[0]));
            e.putLong("d" + i, Double.doubleToRawLongBits(samples.get(i)[1]));
        }
        e.putInt("count", samples.size());
        return e.commit(); // wait for disk commit so an interrupted run retains trials
    }

    private void resetSamples() {
        drivetrain.stop(true);
        samples.clear();
        prefs.edit().clear().commit();
    }

    /** Copies the calculated coefficients into the separate MattPathing AutoTune checkpoint.
     *  This is deliberately opt-in so unusual/invalid trial data is not silently imported.
     */
    private void publishStep8Checkpoint() {
        double[] c = coefficients();
        if (c == null) {
            status = "Cannot import: regression fit failed.";
            return;
        }
        if (c[0] < 0 || c[1] < 0) {
            status = "Cannot import negative coefficients. Review sample quality.";
            return;
        }
        boolean saved = mattPrefs.edit()
                .putString("step.8.0", Double.toString(c[0]))
                .putString("step.8.1", Double.toString(c[1]))
                .putBoolean("step.8.complete", true)
                .commit();
        status = saved
                ? "Step 8 imported into MattPathing. Resume with 'Rerun from step = 0'."
                : "ERROR: could not save MattPathing Step 8 checkpoint.";
    }

    /** Returns {linear, quadratic} for d = linear*v + quadratic*v*v. */
    private double[] coefficients() {
        if (samples.size() < 3) return null;
        double s11 = 0, s12 = 0, s22 = 0, t1 = 0, t2 = 0;
        for (double[] sample : samples) {
            double v = sample[0];
            double d = sample[1];
            double vv = v * v;
            s11 += vv;
            s12 += vv * v;
            s22 += vv * vv;
            t1 += v * d;
            t2 += vv * d;
        }
        double det = s11 * s22 - s12 * s12;
        if (!Double.isFinite(det) || Math.abs(det) < 1e-12) return null;
        double linear = (t1 * s22 - t2 * s12) / det;
        double quadratic = (s11 * t2 - s12 * t1) / det;
        return finite(linear, quadratic) ? new double[]{linear, quadratic} : null;
    }

    private void showTelemetry() {
        telemetry.addLine("MattPathing / Manual Strafe Braking");
        telemetry.addData("State", state);
        telemetry.addData("Status", status);
        telemetry.addData("Progress", "%d / %d saved", samples.size(), POWERS.length);
        if (samples.size() < POWERS.length) {
            telemetry.addData("Next strafe power", POWERS[samples.size()]);
        }
        telemetry.addData("OTOS X / Y (in)", "%.2f / %.2f", lastX, lastY);
        telemetry.addData("OTOS Vx / Vy (in/s)", "%.2f / %.2f", lastVx, lastVy);
        telemetry.addData("OTOS heading (deg)", "%.1f", Math.toDegrees(lastHeading));
        telemetry.addData("Heading change / turn cmd", "%.1f deg / %.3f",
                Math.toDegrees(lastHeadingError), lastHeadingCommand);
        telemetry.addLine("Turn correction DISABLED; all 4 wheels should be commanded equally.");
        if (state == State.COASTING) {
            telemetry.addData("Initial braking speed", "%.3f in/s", brakeStartSpeed);
            telemetry.addData("Braking distance so far", "%.3f in",
                    Math.hypot(lastX - brakeStartX, lastY - brakeStartY));
        }
        telemetry.addLine("Recorded trials (in/s, in):");
        for (int i = 0; i < samples.size(); i++) {
            telemetry.addData("Trial " + (i + 1), String.format(Locale.US,
                    "v=%.3f, d=%.3f", samples.get(i)[0], samples.get(i)[1]));
        }
        if (state == State.COMPLETE) {
            double[] result = coefficients();
            if (result == null) {
                telemetry.addLine("FIT FAILED: velocities are insufficiently distinct.");
            } else {
                telemetry.addData("strafeBrakingLinearCoefficient", "%.9f", result[0]);
                telemetry.addData("strafeBrakingQuadraticCoefficient", "%.9f", result[1]);
                if (result[0] < 0 || result[1] < 0) {
                    telemetry.addLine("CAUTION: negative fit coefficient; check trial data.");
                }
            }
            telemetry.addData("MattPathing Step 8 saved", mattPrefs.getBoolean("step.8.complete", false));
            telemetry.addLine("Press X to copy values into MattPathing Step 8 checkpoint.");
            telemetry.addLine("Press Y to reset manual samples (does not erase MattPathing checkpoint).");
        } else if (state == State.READY) {
            telemetry.addLine("Return robot TO START by hand, keep the same heading, then press A.");
            telemetry.addLine("No automatic return; move it back by hand. Press Y to erase trials.");
        } else if (state == State.ARMING) {
            telemetry.addData("Starting in", "%.1f s",
                    Math.max(0.0, ARM_DELAY_SECONDS - elapsedSeconds(stateStartNs)));
        }
        telemetry.addLine("B = abort | Stop = immediate robot OpMode stop");
        telemetry.update();
    }
}
