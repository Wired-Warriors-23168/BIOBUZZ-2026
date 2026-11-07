package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import android.content.Context;
import android.content.SharedPreferences;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Checkpointed Pedro Pathing 3.x Foresight procedure.
 * Run through the standard AutoTune web interface, not the Driver Station menu.
 * Each successful sub-OpMode is synchronously persisted to Android preferences.
 * A stage is never marked completed if its result is non-finite or invalid.
 */
public class MattPathing extends Procedure {
    private static final String PREF_FILE = "safe_foresight_v1";
    private final SharedPreferences preferences;
    private final Function<HardwareMap, Localizer> localizerFactory;
    private final Function<HardwareMap, Drivetrain> drivetrainFactory;

    public MattPathing(Function<HardwareMap, Localizer> localizerFactory,
                              Function<HardwareMap, Drivetrain> drivetrainFactory,
                              Context context) {
        super("MattPathing",
                "Foresight tuning with motor-safety checks and persistent checkpoints after each step.");
        this.localizerFactory = localizerFactory;
        this.drivetrainFactory = drivetrainFactory;
        this.preferences = context.getApplicationContext()
                .getSharedPreferences(PREF_FILE, Context.MODE_PRIVATE);
    }

    @Override
    public void run() throws InterruptedException {
        Inputs checkpointInput = inputs("Saved progress",
                "0 resumes saved steps; 1-10 discards the selected step and all later steps, then reruns them. " +
                "Start the robot at the required position before each motion test.");
        Inputs.Field<Double> rerunInput = checkpointInput.d("Rerun from step (0 = resume)")
                .withDefault(0.0);
        awaitInputs(checkpointInput);
        double rerunRaw = rerunInput.get();
        if (!Double.isFinite(rerunRaw) || rerunRaw < 0 || rerunRaw > 10 ||
                rerunRaw != Math.floor(rerunRaw)) {
            abort("Rerun from step must be an integer from 0 through 10.");
        }
        int rerun = (int) rerunRaw;
        if (rerun > 0) {
            SharedPreferences.Editor editor = preferences.edit();
            for (int i = rerun; i <= 10; i++) {
                editor.remove("step." + i + ".complete");
                for (int j = 0; j < 4; j++) editor.remove("step." + i + "." + j);
            }
            if (!editor.commit()) abort("Could not reset saved checkpoints.");
        }

        Inputs distanceInput = inputs("Velocity test distance", "Distance in inches for max-velocity tests.");
        Inputs.Field<Double> distance = distanceInput.d("Distance (in)").withDefault(48.0);
        awaitInputs(distanceInput);
        if (!validPositive(distance.get())) abort("Velocity-test distance must be finite and positive.");

        double forwardVelocity = stageDouble(1, "Forward Velocity",
                new SafeForwardVelocity(localizerFactory, drivetrainFactory, distance.get()), true);
        double strafeVelocity = stageDouble(2, "Strafe Velocity",
                new SafeStrafeVelocity(localizerFactory, drivetrainFactory, distance.get()), true);

        Inputs velocityInput = inputs("Deceleration target speed", "Target speed in inches/second.");
        Inputs.Field<Double> targetVelocity = velocityInput.d("Speed (in/s)").withDefault(30.0);
        awaitInputs(velocityInput);
        if (!validPositive(targetVelocity.get())) abort("Deceleration target speed must be finite and positive.");

        double forwardDeceleration = stageDouble(3, "Forward Deceleration",
                new SafeForwardDeceleration(localizerFactory, drivetrainFactory, targetVelocity.get()), true);
        double strafeDeceleration = stageDouble(4, "Strafe Deceleration",
                new SafeStrafeDeceleration(localizerFactory, drivetrainFactory, targetVelocity.get()), true);

        List<Double> headingBraking = stageList(5, "Heading Braking",
                new SafeHeadingBraking(localizerFactory, drivetrainFactory), 2, false, false);
        double headingKp = stageDouble(6, "Heading kP",
                new SafeHeadingTuner(localizerFactory, drivetrainFactory), true);
        double headingLinear = headingBraking.get(0);
        double headingQuadratic = headingBraking.get(1);

        // Steps 7 and 8 are MANUAL: no robot motion is commanded for these stages.
        // Reuse MattPathing checkpoints if present, otherwise ask for the four
        // calibrated values already entered in Constants.java.
        List<Double> forwardBraking = manualBrakingStage(7, "Forward Braking");
        List<Double> strafeBraking = manualBrakingStage(8, "Strafe Braking");

        List<Double> forwardTranslational = stageList(9, "Forward Translational",
                new SafeForwardTranslational(localizerFactory, drivetrainFactory),
                4, true, true, true, true);
        List<Double> strafeTranslational = stageList(10, "Strafe Translational",
                new SafeStrafeTranslational(localizerFactory, drivetrainFactory),
                2, true, true);

        double forwardLinear = forwardBraking.get(0);
        double forwardQuadratic = forwardBraking.get(1);
        double strafeLinear = strafeBraking.get(0);
        double strafeQuadratic = strafeBraking.get(1);
        double forwardPrimary = forwardTranslational.get(0);
        double forwardSecondary = forwardTranslational.get(1);
        double coast = forwardTranslational.get(2);
        double brake = forwardTranslational.get(3);
        double strafePrimary = strafeTranslational.get(0);
        double strafeSecondary = strafeTranslational.get(1);

        result("maxAchievableForwardVelocity", forwardVelocity);
        result("maxAchievableStrafeVelocity", strafeVelocity);
        result("naturalForwardDeceleration", forwardDeceleration);
        result("naturalStrafeDeceleration", strafeDeceleration);
        result("headingBrakingLinearCoefficient", headingLinear);
        result("headingBrakingQuadraticCoefficient", headingQuadratic);
        result("heading kP", headingKp);
        result("forwardBrakingLinearCoefficient", forwardLinear);
        result("forwardBrakingQuadraticCoefficient", forwardQuadratic);
        result("strafeBrakingLinearCoefficient", strafeLinear);
        result("strafeBrakingQuadraticCoefficient", strafeQuadratic);
        result("forwardTranslational Primary kP", forwardPrimary);
        result("forwardTranslational Secondary kP", forwardSecondary);
        result("strafeTranslational Primary kP", strafePrimary);
        result("strafeTranslational Secondary kP", strafeSecondary);
        result("coast kV", coast);
        result("brake kV", brake);

        code(Language.JAVA,
                "public static ForesightConfig foresightConfig = new ForesightConfig(\n" +
                "    c -> {\n" +
                "        Controller primaryForward = Controller.proportional(" + forwardPrimary + ");\n" +
                "        Controller secondaryForward = Controller.proportional(" + forwardSecondary + ");\n" +
                "        Controller primaryLateral = Controller.proportional(" + strafePrimary + ");\n" +
                "        Controller secondaryLateral = Controller.proportional(" + strafeSecondary + ");\n" +
                "        c.forwardTranslational.set(Controller.piecewise(secondaryForward).put(2.5, primaryForward));\n" +
                "        c.strafeTranslational.set(Controller.piecewise(secondaryLateral).put(2.5, primaryLateral));\n" +
                "        c.coast.set(Controller.proportionalFeedforward(" + coast + "));\n" +
                "        c.brake.set(Controller.proportionalFeedforward(" + brake + "));\n" +
                "        c.headingFeedback.set(Controller.proportional(" + headingKp + "));\n" +
                "        c.headingBrakeCoefficients.set(Vector2D.cartesian(" + headingLinear + ", " + headingQuadratic + "));\n" +
                "        c.linearBrakeCoefficients.set(Matrix.diag(" + forwardLinear + ", " + strafeLinear + "));\n" +
                "        c.quadraticBrakeCoefficients.set(Matrix.diag(" + forwardQuadratic + ", " + strafeQuadratic + "));\n" +
                "        c.maxAchievableForwardVelocity.set(" + forwardVelocity + ");\n" +
                "        c.maxAchievableStrafeVelocity.set(" + strafeVelocity + ");\n" +
                "        c.naturalForwardDeceleration.set(" + forwardDeceleration + ");\n" +
                "        c.naturalStrafeDeceleration.set(" + strafeDeceleration + ");\n" +
                "    }\n" +
                ");");
    }

    /**
     * Replaces moving braking identification tests with a checkpoint or a
     * user-entered measurement. The inputs must come from the robot's own
     * manual braking measurements (same distance/speed units).
     * No drive commands are issued by this method.
     */
    private List<Double> manualBrakingStage(int step, String name)
            throws InterruptedException {
        List<Double> saved = load(step, name, false, false);
        if (saved != null) return saved;

        Inputs manual = inputs("Step " + step + ": " + name + " coefficients",
                "No robot movement. Copy your measured LINEAR and QUADRATIC " +
                "braking coefficients from Constants.java. They are not read automatically. " +
                "The values will be saved as a MattPathing checkpoint.");
        Inputs.Field<Double> linear = manual.d("Linear coefficient")
                .withDefault(0.0);
        Inputs.Field<Double> quadratic = manual.d("Quadratic coefficient")
                .withDefault(0.0);
        awaitInputs(manual);
        List<Double> measured = List.of(linear.get(), quadratic.get());
        checkValues(name, measured, false, false);
        if (measured.get(0) == 0.0 && measured.get(1) == 0.0) {
            abort(name + ": enter your measured coefficients; both cannot be zero.");
        }
        save(step, measured);
        return measured;
    }

    private double stageDouble(int step, String name, TuningOpMode<Double> op,
                               boolean positive) throws InterruptedException {
        List<Double> saved = load(step, name, positive);
        if (saved != null) return saved.get(0);
        double value = execute(op);
        checkValues(name, List.of(value), new boolean[]{positive});
        save(step, List.of(value));
        return value;
    }

    private List<Double> stageList(int step, String name, TuningOpMode<List<Double>> op,
                                   int size, boolean... positive) throws InterruptedException {
        if (size != positive.length) throw new IllegalArgumentException("Invalid value validation schema");
        List<Double> saved = load(step, name, positive);
        if (saved != null) return saved;
        List<Double> values = execute(op);
        if (values == null || values.size() != size) abort(name + " returned an unexpected result size.");
        checkValues(name, values, positive);
        save(step, values);
        return values;
    }

    private <T> T execute(TuningOpMode<T> op) throws InterruptedException {
        try {
            return runOpMode(op);
        } catch (RuntimeException ex) {
            abort(op.name + " failed: " + ex.getMessage() + ". Checkpoint was not advanced.");
            throw ex;
        }
    }

    private List<Double> load(int step, String name, boolean... positive) throws InterruptedException {
        String root = "step." + step + ".";
        if (!preferences.getBoolean(root + "complete", false)) return null;
        List<Double> result = new ArrayList<>();
        try {
            for (int i = 0; i < positive.length; i++) {
                String raw = preferences.getString(root + i, null);
                if (raw == null) throw new NumberFormatException("Missing value");
                result.add(Double.parseDouble(raw));
            }
        } catch (RuntimeException ex) {
            abort("Saved checkpoint " + step + " (" + name + ") is incomplete. Rerun from this step.");
        }
        checkValues(name, result, positive);
        return result;
    }

    private void save(int step, List<Double> values) throws InterruptedException {
        String root = "step." + step + ".";
        SharedPreferences.Editor editor = preferences.edit();
        for (int i = 0; i < values.size(); i++)
            editor.putString(root + i, Double.toString(values.get(i)));
        editor.putBoolean(root + "complete", true);
        if (!editor.commit()) abort("Failed to persist checkpoint for step " + step);
    }

    private static boolean validPositive(double d) {
        return Double.isFinite(d) && d > 0;
    }

    private void checkValues(String name, List<Double> values, boolean... positive)
            throws InterruptedException {
        if (values.size() != positive.length) abort("Invalid results in " + name);
        for (int i = 0; i < positive.length; i++) {
            double v = values.get(i);
            if (!Double.isFinite(v) || (positive[i] && v <= 0)) {
                abort("Invalid " + name + " result [" + i + "] = " + v +
                        ". Re-run this stage after checking motion and sensor measurements.");
            }
        }
    }
}
