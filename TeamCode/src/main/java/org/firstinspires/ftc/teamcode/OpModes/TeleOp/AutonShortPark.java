

package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Short Park", group = "Autonomous")
public class AutonShortPark extends LinearOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(9, 128, 0);
    private final Pose point1 = poseFactory.of(13, 124, 0);
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

        // Combine into one continuous path

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

        follower.follow(path1);

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
