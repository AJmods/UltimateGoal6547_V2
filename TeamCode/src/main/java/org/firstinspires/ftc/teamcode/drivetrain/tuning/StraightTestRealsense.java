package org.firstinspires.ftc.teamcode.drivetrain.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class StraightTestRealsense extends LinearOpMode {
    public static double DISTANCE = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        telemetry.log().add("POSE ESTIME: " + bot.getPoseEstimate().toString());

        bot.setPoseEstimate(new Pose2d(-68,-24,0));

        telemetry.log().add("POSE ESTIME 2: " + bot.getPoseEstimate().toString());

        bot.setPoseEstimate(new Pose2d(-68,-24,0));

        telemetry.log().add("POSE ESTIME 3: " + bot.getPoseEstimate().toString());

        Trajectory trajectory = bot.trajectoryBuilder(bot.getPoseEstimate())
                .forward(DISTANCE)
                .build();
        waitForStart();

        if (isStopRequested()) return;

        bot.followTrajectorySync(trajectory);

        sleep(1000);
        bot.followTrajectorySync(bot.trajectoryBuilder().back(DISTANCE).build());
    }
}
