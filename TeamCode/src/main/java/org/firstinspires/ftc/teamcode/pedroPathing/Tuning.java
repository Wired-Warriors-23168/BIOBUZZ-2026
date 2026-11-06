package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.OTOSTuner;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.MattPathing;

/**
 * Pedro Pathing 3.0 AutoTune procedures.
 * <p>
 * Deploy this code, connect to the robot's Wi-Fi and open http://192.168.43.1:10158 in a browser to run them
 * (see https://pedropathing.com/docs/pathing/tuning). Paste the configs that AutoTune generates into
 * {@link Constants}.
 * <p>
 * The tuners below assume a mecanum drivetrain with a goBILDA Pinpoint. If your robot differs, swap in the
 * matching tuner and localizer/drivetrain from the procedures package (e.g. {@code OTOSTuner},
 * {@code ThreeWheelTuner}, {@code TwoWheelTuner}, {@code OctoQuadTuner}).
 */
public class Tuning {

    @Tuner
    public static Procedure mecanumTuner() {

        return new MecanumTuner();
    }

    @Tuner
    public static Procedure otosTuner() {
        return new OTOSTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                (hardwareMap) -> new OTOSLocalizer(hardwareMap, Constants.localizerConfig),
                (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }
    @Tuner
    public static Procedure mattPathing() {
        return new MattPathing(
                hardwareMap -> new OTOSLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                AppUtil.getDefContext()
        );
    }
    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new OTOSLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig)
        );
    }
}
