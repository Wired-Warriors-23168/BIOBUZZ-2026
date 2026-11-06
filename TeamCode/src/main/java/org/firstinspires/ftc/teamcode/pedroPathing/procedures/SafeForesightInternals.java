// Pedro Pathing Quickstart BSD-3-Clause-derived source, pinned to 4f33ef5bbbd99ebfe978f0b86046ca43469e2df2
// Changes: finite motor/sensor guards, stop-on-exception, stop requests,
// watchdog timeouts, and direction-aware braking thresholds.
package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.*;
import java.util.function.Function;

import static com.pedropathing.utils.Utils.linearFit;
import static com.pedropathing.utils.Utils.quadraticFit;



class SafeForwardVelocity extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double distance;
    private final ArrayDeque<Double> velocities = new ArrayDeque<>();
    public static double RECORD_NUMBER = 10;

    public SafeForwardVelocity(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double distance) {
        super("Max Forward Velocity", "A tuner for finding the maximum achievable forward velocity. This will drive forward for " + distance + " inches and then likely drift past that position.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        DrivePowers power = new DrivePowers(1,0,0);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        while (!end && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 4.0, "ForwardVelocity");
            localizer.update();
            SafeForesightChecks.check(localizer);
            if (Math.abs(localizer.pose().x()) > distance) {
                end = true;
                drivetrain.stop();
            } else {
                SafeForesightChecks.drive(drivetrain, power, true);
                double currentVelocity = Math.abs(localizer.twist().toVector2D().x());
                velocities.addLast(currentVelocity);
                velocities.removeFirst();
            }
        }

        drivetrain.stop();
        double average = 0;
        for (double velocity : velocities) {
                average += velocity;
        }
        average /= velocities.size();
        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");
        return average;
    
        } finally {
            drivetrain.stop();
        }
}
}

class SafeStrafeVelocity extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double distance;
    private final ArrayDeque<Double> velocities = new ArrayDeque<>();
    public static double RECORD_NUMBER = 10;

    public SafeStrafeVelocity(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double distance) {
        super("Max Strafe Velocity", "A tuner for finding the maximum achievable strafe velocity. This will drive left for " + distance + " inches and then likely drift past that position.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        DrivePowers power = new DrivePowers(0,1,0);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        while (!end && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 4.0, "StrafeVelocity");
            localizer.update();
            SafeForesightChecks.check(localizer);
            if (Math.abs(localizer.pose().y()) > distance) {
                end = true;
                drivetrain.stop();
            } else {
                SafeForesightChecks.drive(drivetrain, power, false);
                double currentVelocity = Math.abs(localizer.twist().toVector2D().y());
                velocities.addLast(currentVelocity);
                velocities.removeFirst();
            }
        }

        drivetrain.stop();
        double average = 0;
        for (double velocity : velocities) {
            average += velocity;
        }
        average /= velocities.size();
        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");
        return average;
    
        } finally {
            drivetrain.stop();
        }
}
}

class SafeForwardDeceleration extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double velocity;

    private final ArrayList<Double> accelerations = new ArrayList<>();

    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;

    public SafeForwardDeceleration(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double velocity) {
        super("Forward Deceleration", "A tuner for finding the deceleration of the robot when moving forward. This will move forward until it reaches " + velocity + " inches per second.", false);

        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.velocity = velocity;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        accelerations.clear();
        previousVelocity = 0;
        previousTimeNano = 0;
        stopping = false;

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        DrivePowers power = new DrivePowers(1, 0, 0);
        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        SafeForesightChecks.drive(drivetrain, power, false);

        while (!stopping && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 4.0, "ForwardDeceleration");
            if (Math.abs(localizer.pose().x()) > 60) throw new IllegalStateException("Target velocity not reached within 60 inches");
            localizer.update();
            SafeForesightChecks.check(localizer);
            double currentVelocity = localizer.twist().toVector2D().x();
            if (Math.abs(currentVelocity) > velocity) {
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();

                stopping = true;
                drivetrain.stop(false);
            } else {
                SafeForesightChecks.drive(drivetrain, power, false);
            }
        }

        boolean end = false;

        while (!end && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 4.0, "ForwardDeceleration");
            localizer.update();
            SafeForesightChecks.check(localizer);
            drivetrain.stop(false);
            double currentVelocity = localizer.twist().toVector2D().x();
            long currentTimeNano = System.nanoTime();
            double dt = (currentTimeNano - previousTimeNano) / 1e9;

            if (dt > 0) {
                double acceleration = (currentVelocity - previousVelocity) / dt;
                accelerations.add(acceleration);
            }

            previousVelocity = currentVelocity;
            previousTimeNano = currentTimeNano;

            if (Math.abs(currentVelocity) <= 1) {
                end = true;
            }
        }

        drivetrain.stop(false);

        double average = 0;

        for (double acceleration : accelerations) {
            average += acceleration;
        }

        if (accelerations.isEmpty()) {
            if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");
            return 0.0;
        }

        average /= accelerations.size();

        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");

        return Math.abs(average);
    
        } finally {
            drivetrain.stop();
        }
}
}

class SafeStrafeDeceleration extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double velocity;

    private final ArrayList<Double> accelerations = new ArrayList<>();

    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;

    public SafeStrafeDeceleration(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double velocity) {
        super("Strafe Deceleration", "A tuner for finding the deceleration of the robot when moving laterally. This will drive left until it reaches " + velocity + " inches per second.", false);

        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.velocity = velocity;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        accelerations.clear();
        previousVelocity = 0;
        previousTimeNano = 0;
        stopping = false;

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        DrivePowers power = new DrivePowers(0, 1, 0);
        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        SafeForesightChecks.drive(drivetrain, power, false);

        while (!stopping && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 4.0, "StrafeDeceleration");
            if (Math.abs(localizer.pose().y()) > 60) throw new IllegalStateException("Target velocity not reached within 60 inches");
            localizer.update();
            SafeForesightChecks.check(localizer);
            double currentVelocity = localizer.twist().toVector2D().y();
            if (Math.abs(currentVelocity) > velocity) {
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();

                stopping = true;
                drivetrain.stop(false);
            } else {
                SafeForesightChecks.drive(drivetrain, power, false);
            }
        }

        boolean end = false;

        while (!end && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 4.0, "StrafeDeceleration");
            drivetrain.stop(false);
            localizer.update();
            SafeForesightChecks.check(localizer);
            double currentVelocity = localizer.twist().toVector2D().y();
            long currentTimeNano = System.nanoTime();
            double dt = (currentTimeNano - previousTimeNano) / 1e9;

            if (dt > 0) {
                double acceleration = (currentVelocity - previousVelocity) / dt;
                accelerations.add(acceleration);
            }

            previousVelocity = currentVelocity;
            previousTimeNano = currentTimeNano;

            if (Math.abs(currentVelocity) <= 1) {
                end = true;
            }
        }

        drivetrain.stop(false);

        double average = 0;

        for (double acceleration : accelerations) {
            average += acceleration;
        }

        if (accelerations.isEmpty()) {
            if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");
            return 0.0;
        }

        average /= accelerations.size();

        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");

        return Math.abs(average);
    
        } finally {
            drivetrain.stop();
        }
}
}

class SafeHeadingBraking extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    private static double[] POWERS;
    public static double MAX_BRAKE_TIME = 3; //seconds, the robot shouldn't take longer than this to brake

    public static int trials = 12;
    public static double maxPower = 1;
    public static double minPower = 0.2;
    public static double bias = 1.5; // how much it favors doing trials with higher powers
    public static double brakingPower = 0.001;

    private final ElapsedTime timer = new ElapsedTime();

    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;

    private double startHeading;
    private double measuredVelocity;
    private double totalHeading;
    private double previousHeading;
//    private VoltageSensor voltageSensor;

    public SafeHeadingBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Heading Braking", "A tuner for finding the Heading Braking Coefficients. The robot will turn back at forth at various speed levels.", false);

        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        List<Double> coefficients = Collections.emptyList();

        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);
        timer.reset();

        while (state != State.DONE && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 85.0, "HeadingBraking");
            localizer.update();
            SafeForesightChecks.check(localizer);
            double currentHeading = localizer.pose().heading();
            totalHeading += Angle.normalizeSigned(currentHeading - previousHeading);
            previousHeading = currentHeading;

            direction = (iteration % 2 == 0) ? 1 : -1;
            if (iteration < POWERS.length) {
                power = POWERS[iteration];
            }

//            if (state != State.DONE) {
//                double voltage = voltageSensor.getVoltage();
//                double duty = state == State.BRAKE ? -brakingPower * direction: power * direction;
//                double appliedVoltage = voltage * duty;
//            }

            switch (state) {
                case DRIVE: {
                    if (timer.seconds() > 2) {
                        startHeading = totalHeading;
                        measuredVelocity = Math.abs(localizer.velocity().omega);

                        SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, -brakingPower * direction), false);
                        state = State.BRAKE;
                        timer.reset();
                        break;
                    }
                    SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, power * direction), false);
                    break;
                }
                case BRAKE: {
                    if (Math.abs(localizer.velocity().omega) > 0.001 && timer.seconds() < MAX_BRAKE_TIME) {
                        SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, -brakingPower * direction), false);
                        break;
                    }

                    double endHeading = totalHeading;
                    double brakingDistance = Math.abs(endHeading - startHeading);

                    velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

                    iteration++;

                    if (iteration >= POWERS.length) {
                        drivetrain.stop();

                        double[] c = quadraticFit(velocityToBrakingDistance);
                        coefficients = List.of(c[0], c[1]);

                        state = State.DONE;
                    } else {
                        timer.reset();
                        state = State.DRIVE;
                    }
                    break;
                }
                case DONE: {}
            }
        }


        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");


        return coefficients;
    
        } finally {
            drivetrain.stop();
        }
}

    private enum State {
        DRIVE,
        BRAKE,
        DONE
    }

    private static double[] biasedGradient(
            int count,
            double max,
            double min,
            double bias
    ) {
        if (count < 2) return new double[]{  max};

        double[] values = new double[count];

        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);

            double curved = 1 - Math.pow(t, bias);

            values[i] = min + curved * (max - min);
        }

        return values;
    }
}

class SafeHeadingTuner extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    private static final double POWER = 0.4;
    private static final double RUNTIME = 1.2;
    private static final int SAMPLES = 15;
    public static double ALPHA = 18.25;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private double lastTime = 0.0;

    public SafeHeadingTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Heading Tuner", "A tuner for finding the Heading Tuning Coefficients using system identification. This will spin the robot in place for a couple seconds.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        times.clear();
        velocities.clear();
        done = false;
        vMax = 0;
        lastTime = 0.0;

        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);
        timer.reset();
        lastTime = timer.seconds();
        SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, POWER), false);

        while (!done && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 8.0, "HeadingTuner");
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            SafeForesightChecks.check(localizer);

            if (!done) {
                times.add(timer.seconds());

                double turnVel = Math.abs(localizer.velocity().omega);
                vMax = Math.max(vMax, turnVel / POWER);

                velocities.add(turnVel);

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, 0.0), false);
                } else {
                    SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, POWER), false);
                }
            }
        }

        SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, 0.0), true);
        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");
        return calculatekP(ALPHA);
    
        } finally {
            drivetrain.stop();
        }
}

    private double calculatekP(double alpha) {
        kV = 1 / K;
        kA = tau / K;
        return tau * alpha * alpha / K;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        if (x.size() < 2 || !Double.isFinite(K) || K <= 0)
            throw new IllegalArgumentException("Too few usable system identification samples");
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        // With fewer than two samples between 10% and 80% the slope is NaN, which `== 0` let through.
        if (x.size() < 2 || !Double.isFinite(linReg[1]) || linReg[1] >= 0) {
            throw new IllegalArgumentException("Failed calibration: only " + x.size()
                    + " samples while the robot sped up.");
        }
        this.tau = -1.0/linReg[1];
    }
}

class SafeForwardBraking extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    private final double headingLinear;
    private final double headingQuadratic;
    private final double headingKP;

    private double[] POWERS;
    public double MAX_BRAKE_TIME = 7.0;
    public double MAX_DRIVE_SECONDS = 4.0;
    public int trials = 5;
    public double maxPower = 0.7;
    public double minPower = 0.3;
    public double bias = 1.5;
    public double brakingPower = 0.001;
    public double distance;
    public double IDLE_SECONDS = 1;
    /** Forward-axis overshoot is allowed during braking measurement. */
    /** Stops the robot if it drifts this many inches across the test's direction. */
    public double DRIFT = 48;

    private final ElapsedTime timer = new ElapsedTime();
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;
    private Vector2D startPosition;
    private double measuredVelocity;

    public SafeForwardBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction,
                          double headingLinear, double headingQuadratic, double headingKP, double distance) {
        super("Forward Braking", "A tuner for finding the Forward Braking Coefficients by driving forward and backward at various speeds. Please ensure that you have plenty of room at least " + distance + " inches ahead of the robot, but also tile space behind and laterally around the robot.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.headingLinear = headingLinear;
        this.headingQuadratic = headingQuadratic;
        this.headingKP = headingKP;
        this.distance = distance;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        List<Double> coefficients = Collections.emptyList();

        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);
        timer.reset();

        SafeForesightChecks.drive(drivetrain, new DrivePowers(maxPower,0,0), false);

        while (state != State.DONE && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 80.0, "ForwardBraking");
            localizer.update();
            SafeForesightChecks.check(localizer);
            if (Math.abs(localizer.pose().y()) > DRIFT) {
                drivetrain.stop();
                throw new IllegalStateException(String.format(Locale.US,
                        "stopped the robot at (%.0f, %.0f): lateral drift exceeded %.0f inches",
                        localizer.pose().x(), localizer.pose().y(), DRIFT));
            }
            direction = (iteration % 2 == 0) ? 1 : -1;
            if (iteration < POWERS.length) {
                power = POWERS[iteration];
            }

            switch (state) {
                case DRIVE: {
                    if (timer.seconds() > MAX_DRIVE_SECONDS) {
                        drivetrain.stop();
                        throw new IllegalStateException("Drive-phase timeout: " +
                            "direction=" + direction + ", pos=" + localizer.pose());
                    }
                    if ((direction > 0 && localizer.pose().x() >= distance) || (direction < 0 && localizer.pose().x() <= 12)) {
                        startPosition = localizer.pose().toVector2D();
                        measuredVelocity = localizer.velocity().toVector2D().magnitude();

                        brake(drivetrain, localizer);
                        state = State.BRAKE;
                        timer.reset();
                        break;
                    }
                    drive(drivetrain, localizer);
                    break;
                }
                case BRAKE: {
                    if (localizer.velocity().toVector2D().x() * direction > 0.25 && timer.seconds() < MAX_BRAKE_TIME) {
                        brake(drivetrain, localizer);
                        break;
                    }

                    collectTrialData(localizer, drivetrain);
                    break;
                }
                case WAIT: {
                    drivetrain.stop();
                    if (timer.seconds() > IDLE_SECONDS) {
                        timer.reset();
                        state = State.DRIVE;
                    }
                    break;
                }
                case DONE: {}
            }
        }

        if (state == State.DONE) {
            double[] c = quadraticFit(velocityToBrakingDistance);
            coefficients = List.of(c[0], c[1]);
        }

        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");

        return coefficients;
    
        } finally {
            drivetrain.stop();
        }
}

    private double getHeadingPower(Localizer localizer) {
        double angularVel = localizer.velocity().omega;
        double brakeDist = headingLinear * angularVel +
                headingQuadratic * angularVel * angularVel * Math.signum(angularVel);
        double headingError = Angle.normalizeSigned(-localizer.pose().heading());
        double error = headingError - brakeDist;
        return Utils.clamp(headingKP * error, -0.3, 1.0) / 2;
    }


    private void drive(Drivetrain drivetrain, Localizer localizer) {
        SafeForesightChecks.drive(drivetrain, new DrivePowers(power * direction, 0.0, getHeadingPower(localizer)), false);
    }

    private void brake(Drivetrain drivetrain, Localizer localizer) {
        double headingPower = getHeadingPower(localizer);
        double brake = -brakingPower * direction;
        double minBrake = Math.abs(headingPower) + 0.001;

        if (direction > 0) {
            brake = Math.min(brake, -minBrake);
        } else {
            brake = Math.max(brake, minBrake);
        }

        SafeForesightChecks.drive(drivetrain, new DrivePowers(brake, 0, headingPower), false);
    }

    private void collectTrialData(Localizer localizer, Drivetrain drivetrain) {
        Vector2D endPosition = localizer.pose().toVector2D();
        double brakingDistance = endPosition.minus(startPosition).magnitude();

        velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

        iteration++;

        if (iteration >= POWERS.length) {
            drivetrain.stop();
            state = State.DONE;
        } else {
            state = State.WAIT;
            timer.reset();
        }
    }

    private enum State {
        DRIVE,
        BRAKE,
        WAIT,
        DONE
    }

    private static double[] biasedGradient(int count, double max, double min, double bias) {
        if (count < 2) return new double[]{max};
        double[] values = new double[count];
        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);
            double curved = 1 - Math.pow(t, bias);
            values[i] = min + curved * (max - min);
        }
        return values;
    }
}

class SafeStrafeBraking extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    private final double headingLinear;
    private final double headingQuadratic;
    private final double headingKP;

    private double[] POWERS;
    public double MAX_BRAKE_TIME = 7.0;
    public double MAX_DRIVE_SECONDS = 4.0;
    public  int trials = 5;
    public double maxPower = 1;
    public double minPower = 0.3;
    public double bias = 1.5;
    public double brakingPower = 0.001;
    public double distance;
    public double IDLE_SECONDS = 1;
    /** Stops the robot if it gets this many inches past the test's ends. */
    public double OVERRUN = 24;
    /** Stops the robot if it drifts this many inches across the test's direction. */
    public double DRIFT = 48;

    private final ElapsedTime timer = new ElapsedTime();
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;
    private Vector2D startPosition;
    private double measuredVelocity;

    public SafeStrafeBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction,
                         double headingLinear, double headingQuadratic, double headingKP, double distance) {
        super("Strafe Braking", "A tuner for finding the Strafe Braking Coefficients by strafing left and right at various speeds. Please ensure that you have plenty of room at least " + distance + " inches to the left and right of the robot and space in front and behind the robot.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.headingLinear = headingLinear;
        this.headingQuadratic = headingQuadratic;
        this.headingKP = headingKP;
        this.distance = distance;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        List<Double> coefficients = Collections.emptyList();

        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);
        timer.reset();

        SafeForesightChecks.drive(drivetrain, new DrivePowers(0,maxPower,0), false);

        while (state != State.DONE && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 80.0, "StrafeBraking");
            localizer.update();
            SafeForesightChecks.check(localizer);
            if (Math.abs(localizer.pose().y()) > distance + OVERRUN || Math.abs(localizer.pose().x()) > DRIFT) {
                drivetrain.stop();
                throw new IllegalStateException(String.format(Locale.US,
                        "stopped the robot at (%.0f, %.0f): more than %.0f in past the test's ends or %.0f in across them",
                        localizer.pose().x(), localizer.pose().y(), OVERRUN, DRIFT));
            }
            direction = (iteration % 2 == 0) ? 1 : -1;
            if (iteration < POWERS.length) {
                power = POWERS[iteration];
            }

            switch (state) {
                case DRIVE: {
                    if (timer.seconds() > MAX_DRIVE_SECONDS) {
                        drivetrain.stop();
                        throw new IllegalStateException("Drive-phase timeout: " +
                            "direction=" + direction + ", pos=" + localizer.pose());
                    }
                    if ((direction > 0 && localizer.pose().y() >= distance) ||
                            (direction < 0 && localizer.pose().y() <= 6)) {
                        startPosition = localizer.pose().toVector2D();
                        measuredVelocity = localizer.velocity().toVector2D().magnitude();

                        brake(drivetrain, localizer);
                        state = State.BRAKE;
                        timer.reset();
                        break;
                    }
                    drive(drivetrain, localizer);
                    break;
                }
                case BRAKE: {
                    if (localizer.velocity().toVector2D().y() * direction > 0.25 && timer.seconds() < MAX_BRAKE_TIME) {
                        brake(drivetrain, localizer);
                        break;
                    }

                    collectTrialData(localizer, drivetrain);
                    break;
                }
                case WAIT: {
                    drivetrain.stop();
                    if (timer.seconds() > IDLE_SECONDS) {
                        timer.reset();
                        state = State.DRIVE;
                    }
                    break;
                }
                case DONE: {}
            }
        }

        if (state == State.DONE) {
            double[] c = quadraticFit(velocityToBrakingDistance);
            coefficients = List.of(c[0], c[1]);
        }

        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");

        return coefficients;
    
        } finally {
            drivetrain.stop();
        }
}

    private double getHeadingPower(Localizer localizer) {
        double angularVel = localizer.velocity().omega;
        double brakeDist = headingLinear * angularVel +
                headingQuadratic * angularVel * angularVel * Math.signum(angularVel);
        double headingError = Angle.normalizeSigned(-localizer.pose().heading());
        double error = headingError - brakeDist;
        return Utils.clamp(headingKP * error, -0.3, 1.0) / 2;
    }

    private void drive(Drivetrain drivetrain, Localizer localizer) {
        SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, power * direction, getHeadingPower(localizer)), false);
    }

    private void brake(Drivetrain drivetrain, Localizer localizer) {
        double headingPower = getHeadingPower(localizer);
        double brake = -brakingPower * direction;
        double minBrake = Math.abs(headingPower) + 0.001;

        if (direction > 0) {
            brake = Math.min(brake, -minBrake);
        } else {
            brake = Math.max(brake, minBrake);
        }

        SafeForesightChecks.drive(drivetrain, new DrivePowers(0, brake, headingPower), false);
    }

    private void collectTrialData(Localizer localizer, Drivetrain drivetrain) {
        Vector2D endPosition = localizer.pose().toVector2D();
        double brakingDistance = endPosition.minus(startPosition).magnitude();

        velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

        iteration++;

        if (iteration >= POWERS.length) {
            drivetrain.stop();
            state = State.DONE;
        } else {
            state = State.WAIT;
            timer.reset();
        }
    }

    private enum State {
        DRIVE,
        BRAKE,
        WAIT,
        DONE
    }

    private static double[] biasedGradient(int count, double max, double min, double bias) {
        if (count < 2) return new double[]{max};
        double[] values = new double[count];
        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);
            double curved = 1 - Math.pow(t, bias);
            values[i] = min + curved * (max - min);
        }
        return values;
    }
}

class SafeForwardTranslational extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public static double ALPHA_LARGE = 10.2;
    public static double ALPHA_SMALL = 6.2;
    private final double VEL_AGGRESSIVENESS = 0.85;
    private final double POWER = 0.4;
    private final double RUNTIME = 1.2;
    private final int SAMPLES = 15;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private double lastTime = 0.0;

    public SafeForwardTranslational(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Forward Translational", "A tuner for finding the Forward Translational kP coefficients using system identification. This will move around 12-24 inches in front of the robot and then stop.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        times.clear();
        velocities.clear();
        done = false;
        vMax = 0;
        lastTime = 0.0;

        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);
        timer.reset();
        lastTime = timer.seconds();
        SafeForesightChecks.drive(drivetrain, new DrivePowers(POWER, 0.0, 0.0), false);

        while (!done && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 8.0, "ForwardTranslational");
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            SafeForesightChecks.check(localizer);

            if (!done) {
                times.add(timer.seconds());

                double forwardVelocity = Math.abs(localizer.twist().toVector2D().x());
                vMax = Math.max(vMax, forwardVelocity / POWER);

                velocities.add(forwardVelocity);

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, 0.0), false);
                } else {
                    SafeForesightChecks.drive(drivetrain, new DrivePowers(POWER, 0.0, 0.0), false);
                }
            }
        }

        SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, 0.0), true);

        double kP_large = calculatekP(ALPHA_LARGE);
        double kP_small = calculatekP(ALPHA_SMALL);

        //  kP_large, kP_small, coast kV, and brake kV (scaled by aggressiveness factor)
        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");
        return List.of(kP_large, kP_small, kV, kV * VEL_AGGRESSIVENESS);
    
        } finally {
            drivetrain.stop();
        }
}

    private double calculatekP(double alpha) {
        kV = 1 / K;
        kA = tau / K;
        return tau * alpha * alpha / K;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        if (x.size() < 2 || !Double.isFinite(K) || K <= 0)
            throw new IllegalArgumentException("Too few usable system identification samples");
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        // With fewer than two samples between 10% and 80% the slope is NaN, which `== 0` let through.
        if (x.size() < 2 || !Double.isFinite(linReg[1]) || linReg[1] >= 0) {
            throw new IllegalArgumentException("Failed calibration: only " + x.size()
                    + " samples while the robot sped up.");
        }
        this.tau = -1.0/linReg[1];
    }
}

class SafeStrafeTranslational extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    public static double ALPHA_LARGE = 10.2;
    public static double ALPHA_SMALL = 6.2;
    private final double POWER = 0.4;
    private final double RUNTIME = 1.2;
    private final int SAMPLES = 15;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private double lastTime = 0.0;

    public SafeStrafeTranslational(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Strafe Translational", "A tuner for finding the Strafe Translational kP coefficients using system identification. This will move around 12-24 inches to the left and right of the robot and then stop.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        try {

        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);

        times.clear();
        velocities.clear();
        done = false;
        vMax = 0;
        lastTime = 0.0;

        Thread.sleep(1000);
        waitForStart();
        long safetyStartNanos = System.nanoTime();
        localizer.setPose(Pose.zero());
        localizer.update();
        SafeForesightChecks.check(localizer);
        timer.reset();
        lastTime = timer.seconds();
        SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, POWER, 0.0), false);

        while (!done && !isStopRequested()) {
            SafeForesightChecks.timeout(safetyStartNanos, 8.0, "StrafeTranslational");
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            SafeForesightChecks.check(localizer);

            if (!done) {
                times.add(timer.seconds());

                double lateralVelocity = Math.abs(localizer.twist().toVector2D().y());
                vMax = Math.max(vMax, lateralVelocity / POWER);

                velocities.add(lateralVelocity);

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, 0.0), false);
                } else {
                    SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, POWER, 0.0), false);
                }
            }
        }

        SafeForesightChecks.drive(drivetrain, new DrivePowers(0.0, 0.0, 0.0), true);

        double kP_large = calculatekP(ALPHA_LARGE);
        double kP_small = calculatekP(ALPHA_SMALL);

        if (isStopRequested()) throw new IllegalStateException("Tuning cancelled by Driver Station");

        return List.of(kP_large, kP_small);
    
        } finally {
            drivetrain.stop();
        }
}

    private double calculatekP(double alpha) {
        kV = 1 / K;
        kA = tau / K;
        return tau * alpha * alpha / K;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        if (x.size() < 2 || !Double.isFinite(K) || K <= 0)
            throw new IllegalArgumentException("Too few usable system identification samples");
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        // With fewer than two samples between 10% and 80% the slope is NaN, which `== 0` let through.
        if (x.size() < 2 || !Double.isFinite(linReg[1]) || linReg[1] >= 0) {
            throw new IllegalArgumentException("Failed calibration: only " + x.size()
                    + " samples while the robot sped up.");
        }
        this.tau = -1.0/linReg[1];
    }
}

