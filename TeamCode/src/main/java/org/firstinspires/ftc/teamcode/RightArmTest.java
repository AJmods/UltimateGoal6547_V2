package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.Bot2;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@TeleOp
@Config
public class RightArmTest extends LinearOpMode {

    public static double SERVO_POS=0;
    @Override
    public void runOpMode() throws InterruptedException {
        Bot2 bot = new Bot2(this);

        telemetry.log().add("Ready to Start");
        waitForStart();

        while (opModeIsActive()) {
            bot.rightArm.setPosition(SERVO_POS);
        }


    }
}
