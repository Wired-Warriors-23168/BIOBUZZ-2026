package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.0 constants. The values below are placeholders: run the AutoTune procedures in
 * {@link Tuning} (see https://pedropathing.com/docs/pathing/tuning) and paste the generated configs here.
 * If you use a different drivetrain or localizer, swap the config classes and {@link #createFollower} to match.
 */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("otos");
        c.linearScalar.set(0.9784654812637766);
        c.angularScalar.set(0.9961316603461035);
        c.offset.set(new Pose(0.16820712352362205, -0.018022191806102362, Math.PI));
        c.linearUnit.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryForward = Controller.proportional(0.41670872214239907);
                Controller secondaryForward = Controller.proportional(0.1539627381694908);
                Controller primaryLateral = Controller.proportional(0.6336262874162778);
                Controller secondaryLateral = Controller.proportional(0.23410798239409575);
                c.forwardTranslational.set(Controller.piecewise(secondaryForward).put(2.5, primaryForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryLateral).put(2.5, primaryLateral));
                c.coast.set(Controller.proportionalFeedforward(0.012086716970138479));
                c.brake.set(Controller.proportionalFeedforward(0.010273709424617706));
                c.headingFeedback.set(Controller.proportional(.55));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.8893876196145848, -0.3058799535103024));
                c.linearBrakeCoefficients.set(Matrix.diag(0.122461629, 0.052941006));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002392376, 0.003078965));
                c.maxAchievableForwardVelocity.set(73.92014147685475);
                c.maxAchievableStrafeVelocity.set(32.1646529306643);
                c.naturalForwardDeceleration.set(30.683721262873863);
                c.naturalStrafeDeceleration.set(75.54419156425315);
            }
    );
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new OTOSLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
