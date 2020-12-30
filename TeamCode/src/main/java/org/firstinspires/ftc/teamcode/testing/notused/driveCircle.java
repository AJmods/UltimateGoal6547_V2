package org.firstinspires.ftc.teamcode.testing.notused;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.drivetrain.DriveSpeeds;

@Autonomous
@Config
public class driveCircle extends LinearOpMode {

    public static double RADIUS =20;
    public static double LOOPS=1;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        waitForStart();

        //bot.setPoseEstimate(new Pose2d(0,RADIUS, 0));

        bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.slow).splineTo(new Vector2d(0, RADIUS),Math.toRadians(360)).build());

        TrajectoryBuilder builder = bot.trajectoryBuilder(false, DriveSpeeds.medium);

        for (int i = 0; i < LOOPS; i++)
        builder = builder.splineTo(new Vector2d(RADIUS, 0),Math.toRadians(270))
                .splineTo(new Vector2d(0, -RADIUS),Math.toRadians(180))
                .splineTo(new Vector2d(-RADIUS, 0),Math.toRadians(90))
                .splineTo(new Vector2d(0, RADIUS),Math.toRadians(360));

        bot.followTrajectorySync(builder.build());

        builder = bot.trajectoryBuilder(true);
        for (int i = 0; i < LOOPS; i++)
            builder = builder.splineTo(new Vector2d(-RADIUS, 0),Math.toRadians(270))
                    .splineTo(new Vector2d(0, -RADIUS),Math.toRadians(0))
                    .splineTo(new Vector2d(RADIUS, 0),Math.toRadians(90))
                    .splineTo(new Vector2d(0, RADIUS),Math.toRadians(180));

        //builder = bot.trajectoryBuilder();

        //drive circle while rotating
//        for (int i = 0; i < LOOPS; i++)
//            builder = builder.splineToSplineHeading(new Pose2d(RADIUS, 0,Math.toRadians(90)), Math.toRadians(270))
//                    .splineToSplineHeading(new Pose2d(0, -RADIUS, Math.toRadians(180)), Math.toRadians(180))
//                    .splineToSplineHeading(new Pose2d(-RADIUS, 0, Math.toRadians(270)), Math.toRadians(90))
//                    .splineToSplineHeading(new Pose2d(0, RADIUS, Math.toRadians(360)), Math.toRadians(360));

        //drive circle backwards (workaround)

        bot.followTrajectorySync(builder.build());

//        builder = bot.trajectoryBuilder();
//
//        for (int i = 0; i < LOOPS; i++)
//            builder = builder.splineToSplineHeading(new Pose2d(-RADIUS, 0, Math.toRadians(270)), Math.toRadians(270))
//                    .splineToSplineHeading(new Pose2d(0, -RADIUS, Math.toRadians(360)), Math.toRadians(360))
//                    .splineToSplineHeading(new Pose2d(RADIUS, 0,Math.toRadians(90)), Math.toRadians(90))
//                    .splineToSplineHeading(new Pose2d(0, RADIUS, Math.toRadians(180)), Math.toRadians(180));
//
//
//        bot.followTrajectorySync(builder.build());

        bot.followTrajectorySync(bot.trajectoryBuilder().splineTo(new Vector2d(0,0),0).build());



    }
}
