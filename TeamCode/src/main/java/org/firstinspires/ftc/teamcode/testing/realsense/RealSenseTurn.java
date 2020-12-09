package org.firstinspires.ftc.teamcode.testing.realsense;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@Autonomous
@Config
@Disabled
public class RealSenseTurn extends LinearOpMode {

    public static double ANGLE=90;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        telemetry.log().add("ready to start");
        waitForStart();

        Pose2d pose2d = bot.getPoseEstimate();

        bot.followTrajectorySync(bot.trajectoryBuilder().splineTo(new Vector2d(pose2d.getX()+1,pose2d.getY()+1), Math.toRadians(ANGLE)).build());
    }
}
