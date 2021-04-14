package org.firstinspires.ftc.teamcode.drivetrain.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTestRealsense extends LinearOpMode {
    public static double ANGLE = 90; // deg
    public static double LOOPS = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        waitForStart();

        if (isStopRequested()) return;

        for (int i = 0; i < LOOPS; i++)
        bot.turnSync(Math.toRadians(ANGLE));
    }
}
