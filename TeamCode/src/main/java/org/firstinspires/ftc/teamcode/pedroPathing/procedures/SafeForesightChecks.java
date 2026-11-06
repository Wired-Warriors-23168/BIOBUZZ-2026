package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;

/** Guards against NaN/stale motor commands, sensor discontinuities and stuck test loops. */
final class SafeForesightChecks {
    private SafeForesightChecks() {}

    static void drive(Drivetrain drivetrain, DrivePowers p, boolean manual) {
        if (!Double.isFinite(p.forward()) ||
                !Double.isFinite(p.strafe()) || !Double.isFinite(p.turn())) {
            drivetrain.stop();
            throw new IllegalStateException("Rejected non-finite motor command: " + p);
        }
        drivetrain.drive(p, manual);
    }

    static void check(Localizer l) {
        if (l.pose() == null ||
                !Double.isFinite(l.pose().x()) ||
                !Double.isFinite(l.pose().y()) ||
                !Double.isFinite(l.pose().heading()) ||
                !Double.isFinite(l.velocity().omega) ||
                !Double.isFinite(l.velocity().toVector2D().x()) ||
                !Double.isFinite(l.velocity().toVector2D().y()) ||
                !Double.isFinite(l.twist().toVector2D().x()) ||
                !Double.isFinite(l.twist().toVector2D().y())) {
            throw new IllegalStateException("Invalid localizer pose/velocity; stopping test");
        }
    }

    static void timeout(long startNanos, double maxSeconds, String phase) {
        if ((System.nanoTime() - startNanos) / 1.0e9 > maxSeconds)
            throw new IllegalStateException("Timeout in " + phase + " (" + maxSeconds + " s)");
    }
}
