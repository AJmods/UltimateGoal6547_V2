package org.firstinspires.ftc.teamcode.drivetrain.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Config
@Disabled
public class SplineTestRealsense extends LinearOpMode {

    public static double X = 30;
    public static double Y = 30;
    public static boolean REVERSE = true;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense drive = new DriveTrain6547Realsense(this);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Vector2d(X, Y), 0)
                        .build()
        );

        sleep(500);

        if (!REVERSE) drive.turnRelativeSync(Math.toRadians(225));



//        drive.followTrajectorySync(
//                drive.trajectoryBuilder(REVERSE)
//                        .splineTo(new Vector2d(0,0),Math.toRadians(180))
//                        .build()
//        );
        TrajectoryBuilder builder = drive.trajectoryBuilder(true);

        drive.followTrajectorySync(builder.splineTo(new Vector2d(0,0), Math.toRadians(180)).build());

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Vector2d(X, Y), 0)
                        .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
        .splineToLinearHeading(new Pose2d(0,0,0),Math.toRadians(180)).build());

        sleep(500);
        if (!REVERSE) drive.turnSync(Math.toRadians(180));
    }
}
