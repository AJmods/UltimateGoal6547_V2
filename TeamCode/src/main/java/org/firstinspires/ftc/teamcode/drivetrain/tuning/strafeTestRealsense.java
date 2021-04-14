package org.firstinspires.ftc.teamcode.drivetrain.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@Autonomous
@Config
public class strafeTestRealsense extends LinearOpMode {

    public static double DISTANCE=24;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder().strafeLeft(DISTANCE).build());

        sleep(1000);

        bot.followTrajectorySync(bot.trajectoryBuilder().strafeRight(DISTANCE).build());
    }
}
