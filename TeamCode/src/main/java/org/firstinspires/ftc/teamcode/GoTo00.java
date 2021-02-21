package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.drivetrain.DriveSpeeds;

/**
 * Tries to make the robot go to (0,0).
 */
@Autonomous(name = "Go to (0,0)")
@Disabled
public class GoTo00 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this, false);

        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.slow)
        .splineToSplineHeading(new Pose2d(0, 0, 0), 0)
                .build());
    }
}
