package org.firstinspires.ftc.teamcode.z_oldFiles.roadRunner.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.z_oldFiles.roadRunner.drive.DriveTrain6547Offseason;

@Autonomous(group = "drive")
@Disabled
public class strafeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Offseason bot = new DriveTrain6547Offseason(this);

        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder()
        .strafeLeft(24)
        .build());
    }
}
