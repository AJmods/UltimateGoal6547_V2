package org.firstinspires.ftc.teamcode.oldFiles.roadRunner.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldFiles.roadRunner.drive.DriveTrain6547Offseason;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Offseason drive = new DriveTrain6547Offseason(this);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Vector2d(30, 30), 0)
                        .build()
        );

        sleep(2000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder(true)
                        //.reverse()
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}
