package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drivetrain.Bot2;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@TeleOp
@Config
public class RightArmTest extends LinearOpMode {

    public static double SERVO_POS=0;
    public static double SERVO_POS_LEFT = 0;

    Servo leftServo;
    @Override
    public void runOpMode() throws InterruptedException {
        Bot2 bot = new Bot2(this);

        leftServo = hardwareMap.get(Servo.class, "leftServo");

        telemetry.log().add("Ready to Start");
        waitForStart();

        while (opModeIsActive()) {
            bot.rightArm.setPosition(SERVO_POS);
            leftServo.setPosition(SERVO_POS_LEFT);
        }


    }
}
