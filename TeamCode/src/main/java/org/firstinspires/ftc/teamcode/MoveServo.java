package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@Config
@TeleOp
@Disabled
public class MoveServo extends LinearOpMode {

    public static double servoPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        while (opModeIsActive()) {
            bot.distanceSensorServoX.setPosition(servoPos);
            bot.distanceSensorServoY.setPosition(servoPos);
        }
    }
}
