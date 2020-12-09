package org.firstinspires.ftc.teamcode.testing.realsense;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.drivetrain.localizer.T265LocalizerRR;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.homar.ToggleBoolean;

/*
This is the tele-op we use to drive the robot
 */
@Config
@TeleOp(name = "SkyStone Tele-op Road Runner RealSense", group = "_teleOp")
@Disabled
public class RoadRunnerRealSenseTest extends LinearOpMode {

    public static double TARGET_ANGLE = 0;
    public static double LEEWAY = 1;
    public static boolean InfinteTurn = false;
    public static double turnTowardX = 0;
    public static double turnTowardY = 0;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    private ToggleBoolean feildRealtive = new ToggleBoolean(true);

    private DriveTrain6547Realsense bot; //the robot class

    private double angleZeroValue = 0;

    private double robotAngle=0;

    @Override
    public void runOpMode() {
       // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //makes telemetry output to the FTC Dashboard
        bot = new DriveTrain6547Realsense(this);
        telemetry.update();

        bot.setPoseEstimate(new Pose2d());

        telemetry.log().add("DONE INITIALING");
        telemetry.log().add("Ready to start");

        waitForStart();

        double speedModifer=1;

        while (opModeIsActive()) {

            bot.updateGamepads();
            Pose2d pos = bot.getPoseEstimate();

            /*
            Speed Modifers
             */
            if (bot.x1.onPress()) speedModifer=.60;
            if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifer=.9;
            if (bot.a1.onPress() && !bot.start1.isPressed()) speedModifer=1.3; //trig math caps speed at .7, 1.3 balences it out

            if (bot.y1.onPress()) feildRealtive.toggle(); //toggle field realtive

            robotAngle = T265LocalizerRR.getHeading() - angleZeroValue; //angle of robot

            if (feildRealtive.output()) //if field relative is enabled
            {
                double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
                double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //get angle
                double rightX=gamepad1.right_stick_x*2; //rotation
                rightX*=.5; //half rotation value for better turning
                //offset the angle by the angle of the robot to make it field realtive
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
            leftFrontPower*=speedModifer;
            leftBackPower*=speedModifer;
            rightBackPower*=speedModifer;
            rightFrontPower*=speedModifer;

            telemetry.addData("leftFront Power", leftFrontPower);

            bot.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

//            if (bot.rightBumper1.onPress() && bot.leftBumper1.onPress()) //calibrate gyro
//            {
//                //double zeroVal = -Math.toDegrees(bot.getRawExternalHeading());
//                angleZeroValue = T265Localizer.getHeading();
//                telemetry.log().add("Calibrated, set zero value to " + angleZeroValue);
//            }

            if (gamepad1.left_stick_button) {
//                double dist = 60; //inches
//                double xDist = Math.cos(pos.getHeading()) * dist;
//                double yDist = Math.sin(pos.getHeading()) * dist;
//                turnTowardX = xDist + pos.getX();
//                turnTowardY = yDist + pos.getY();
               // bot.setMotorPowers(0,0,0,0);
               // sleep(100);
            }

            telemetry.update();

//            if (InfinteTurn) {
//
//               // double targetAngleRad = Math.toRadians(TARGET_ANGLE);
//                double deltaX = turnTowardX - pos.getX();
//                double deltaY = turnTowardY - pos.getY();
//                double targetAngleRad = Math.atan2(deltaY, deltaX);
//                //if (targetAngleRad != lastAngle) {
//                targetAngleRad-= T265LocalizerRR.getHeading();
//                if (Math.abs(targetAngleRad)>Math.toRadians(180)) //make the angle difference less then 180 to remove unnecessary turning
//                {
//                    targetAngleRad+=(targetAngleRad>=0) ? Math.toRadians(-360) : Math.toRadians(360);
//                }
//                try {
//                    bot.turn(targetAngleRad, bot.getCurrentHeadingV(), bot.getCurrentHeadingA(), 0);
//                } catch (Exception e) {
//                    e.printStackTrace();
//                    RobotLog.v("Target Angle:" + targetAngleRad + "Current V: " + bot.getCurrentHeadingV() + ", current A: " + bot.getCurrentHeadingA());
//                    bot.turn(targetAngleRad);
//                }
//                bot.setMotorPowersToAdd(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
//            } else {
//                //set motor powers based on previous calculations
//                bot.mode = DriveTrain6547Realsense.Mode.IDLE;
//                bot.clearMotorPowersToAdd();
//                bot.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
//            }

            bot.update(); //updates robot's position
            /*
            Telemetry
             */
            //Pose2d pos = bot.getPoseEstimate();
            //bot.setPoseEstimate(new Pose2d(pos.getX(), pos.getY(), bot.getRawExternalHeading()+Math.toRadians(90)));


        }
        bot.stopRobot();
    }


}
