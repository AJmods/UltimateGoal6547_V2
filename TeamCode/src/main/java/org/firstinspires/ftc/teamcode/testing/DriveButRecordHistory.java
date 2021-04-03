package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivetrain.Bot2;

import java.util.ArrayList;
import java.util.List;

public class DriveButRecordHistory extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();

    List<Pose2d> poseHistory = new ArrayList<>();
    List<Double> times = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        Bot2 bot2 = new Bot2(this);

        bot2.setPoseEstimate(new Pose2d(-56,0));

        telemetry.log().add("Ready to Start");
        waitForStart();

        time.reset();
        bot2.followTrajectory(bot2.trajectoryBuilder().lineToLinearHeading(new Pose2d(60,0,0)).build());
        while (bot2.isBusy()) {
            bot2.update();
            poseHistory.add(bot2.getPoseEstimate());
            times.add(time.milliseconds());
        }

        bot2.stopRobot();

        bot2.savePoseHistroy(poseHistory, times);

        bot2.readPoseHistory();
    }
}
