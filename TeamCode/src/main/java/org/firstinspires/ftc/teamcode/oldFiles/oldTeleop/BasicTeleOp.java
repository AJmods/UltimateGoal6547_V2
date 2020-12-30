package org.firstinspires.ftc.teamcode.oldFiles.oldTeleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.util.homar.ToggleBoolean;

/*
This is the tele-op we use to drive the robot
 */
@Config
@TeleOp(name = "Basic Tele-op", group = "_teleOp")
public class BasicTeleOp extends LinearOpMode {

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    private final ToggleBoolean failedRelative = new ToggleBoolean(true);

    private DriveTrain6547Realsense bot; //the robot class

    private double angleZeroValue = 0;

    private double robotAngle=0;

    @Override
    public void runOpMode() throws InterruptedException {
       // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //makes telemetry output to the FTC Dashboard
        DriveTrain6547Realsense.USE_REALSENSE = false;
        bot = new DriveTrain6547Realsense(this);
        telemetry.update();

        bot.setPoseEstimate(new Pose2d());

        telemetry.log().add("DONE INITIALING");
        telemetry.log().add("Ready to start");

        waitForStart();

        double speedModifier=1;

        while (opModeIsActive()) {

            bot.updateGamepads();
            Pose2d pos = bot.getPoseEstimate();

            /*
            Speed Modifiers
             */
            if (bot.x1.onPress()) speedModifier=.60;
            if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifier=.9;
            if (bot.a1.onPress() && !bot.start1.isPressed()) speedModifier=1.3; //trig math caps speed at .7, 1.3 balances it out

            if (bot.y1.onPress()) failedRelative.toggle(); //toggle field relative

            robotAngle =  bot.getRawExternalHeading() - angleZeroValue; //angle of robot

            if (failedRelative.output()) //if field relative is enabled
            {
                double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
                double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //get angle
                double rightX=gamepad1.right_stick_x*2; //rotation
                rightX*=.5; //half rotation value for better turning
                //offset the angle by the angle of the robot to make it field relative
                leftFrontPower =  speed * Math.cos(LeftStickAngle-robotAngle) + rightX;
                rightFrontPower =  speed * Math.sin(LeftStickAngle-robotAngle) - rightX;
                leftBackPower =  speed * Math.sin(LeftStickAngle-robotAngle) + rightX;
                rightBackPower =  speed * Math.cos(LeftStickAngle-robotAngle) - rightX;

                telemetry.addData("LS angle",Math.toDegrees(LeftStickAngle));
                telemetry.addData("driving toward",LeftStickAngle-robotAngle);
                telemetry.addData("ROBOT ANGLE",Math.toDegrees(robotAngle));
                telemetry.addData("RAW ANGLE", Math.toDegrees(bot.getRawExternalHeading()));
            }
            else //regular drive (different math because this is faster than sins and cosines
            {
                leftFrontPower=-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                rightFrontPower=-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
                leftBackPower=-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                rightBackPower=-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            }
            leftFrontPower*=speedModifier;
            leftBackPower*=speedModifier;
            rightBackPower*=speedModifier;
            rightFrontPower*=speedModifier;

            telemetry.addData("leftFront Power", leftFrontPower);

            bot.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

            if (bot.rightBumper1.onPress() && bot.leftBumper1.onPress()) //calibrate gyro
            {
                //double zeroVal = -Math.toDegrees(bot.getRawExternalHeading());
                angleZeroValue = bot.getRawExternalHeading();
                telemetry.log().add("Calibrated, set zero value to " + angleZeroValue);
            }

            telemetry.addData("Positions: ", bot.getWheelPositions());

            telemetry.update();
//
            bot.update(); //updates robot's position

        }
        bot.stopRobot();
    }


}
