

package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.follower.Follower;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OTOSLocalizer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

@Autonomous(name = "Long Park", group = "Autonomous")
public class AutonLongPark extends LinearOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 9, 90);
    private final Pose point1 = poseFactory.of(22.1258, 50.5924, 90);
    private final Pose point2 = poseFactory.of(14.1214, 89.2964, 101.6846);
    boolean isRamen = true;

 //   Feeder feeder = new Feeder(hardwareMap, isRamen);
 //   Shooter flywheel = new Shooter(hardwareMap, isRamen);
 //   Hood hood = new Hood(hardwareMap, isRamen);

    @Override
    public void runOpMode() {

        // Initialize follower using existing tuned configurations
        Follower follower = new Follower(
                new OTOSLocalizer(hardwareMap, Constants.localizerConfig),
                new Mecanum(hardwareMap, Constants.drivetrainConfig),
                new Foresight(Constants.foresightConfig)
        );

        // Create paths
        Path path1 = line(start, point1).constant(point1);
        Path path2 = line(point1, point2).tangent();

        // Combine into one continuous path
        Path fullPath = path(path1, path2);

        // Set the starting position
        follower.setPose(start);

        telemetry.addLine("Autonomous initialized");
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            follower.stop();
            follower.update();
            return;
        }

     //   flywheel.flywheelon();
     //   flywheel.shootPollen();
     //   hood.pollenPos();
     //   sleep(2000);
     //   feeder.on();
     //   feeder.gateon();
     //   sleep(2000);
     //   flywheel.flywheeloff();
     //   feeder.feederOff();
     //   feeder.gateoff();


        // Begin autonomous
        follower.follow(fullPath);

        ElapsedTime timer = new ElapsedTime();

        try {
            while (opModeIsActive()
                    && follower.following()
                    && timer.seconds() < 25.0) {

                // Update localization and drivetrain
                follower.update();

                // Display robot position
                telemetry.addData("X", follower.pose().x());
                telemetry.addData("Y", follower.pose().y());
                telemetry.addData("Heading",
                        Math.toDegrees(follower.pose().heading()));
                telemetry.update();

                idle();
            }
        } finally {
            follower.stop();
            follower.update();
        }
    }
}
