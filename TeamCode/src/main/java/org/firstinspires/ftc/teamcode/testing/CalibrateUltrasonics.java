package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.teleOp.LeagueChampionshipTeleop;

@Disabled
public class CalibrateUltrasonics extends LinearOpMode {

    DriveTrain6547Realsense bot;
    LeagueChampionshipTeleop teleop;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = new DriveTrain6547Realsense(this);
        teleop = new LeagueChampionshipTeleop(this, bot);

        bot.initVufoira();

        telemetry.log().add("Ready to start");
        waitForStart();

        bot.startVuforia();

        while (opModeIsActive()) {
            teleop.doTeleOp();

            telemetry.update();
            bot.update();
        }


    }

    public void calibrateUltrasonics() {}


}
