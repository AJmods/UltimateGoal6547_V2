package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@Config
@TeleOp
@Disabled
public class WobGrabTest extends LinearOpMode {

    public static double SERVO_POS = .5;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        telemetry.log().add("ready to start");

        waitForStart();

        while (opModeIsActive()) {
            bot.wobbleGoalGrabber.setPosition(SERVO_POS);
        }

    }
}
