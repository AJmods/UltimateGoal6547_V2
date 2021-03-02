package org.firstinspires.ftc.teamcode.testing.realsense;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.z_oldFiles.roadRunner.drive.DriveTrain6547Offseason;
import org.firstinspires.ftc.teamcode.util.homar.ToggleBoolean;
import org.firstinspires.ftc.teamcode.util.homar.ToggleDouble;

/*
This is the tele-op we use to drive the robot
 */
@Config
@TeleOp(name = "SkyStone Tele-op Off Realsense", group = "_teleOp")
@Disabled
public class TeleOpRealSense extends LinearOpMode {

    public static double slideSpeed = .0045; //speed of horizontal slide in servo position units

    public static double speedModifier =.7; //lowers the speed so it's easier to drive

    private boolean intake = false;
    private boolean outtake = false;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    private final ToggleBoolean fieldRelative = new ToggleBoolean(true);

    //edit the array to change the foundation grabber position(s)
    private final ToggleDouble fondationGrabberPos = new ToggleDouble(new double[] {0,1},0);
    private final ToggleDouble grabberToggle = new ToggleDouble(new double[] {0, 1}, 0);

    private DriveTrain6547Offseason bot; //the robot class

    private static T265Camera slamra;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //makes telemetry output to the FTC Dashboard
        bot = new DriveTrain6547Offseason(this);
        telemetry.update();

        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(new Translation2d(0,0), new Rotation2d(0)), 0, hardwareMap.appContext);
            RobotLog.d("Created Camera");
        }

        bot.disableEncoders();

        telemetry.log().add("DONE INITIALING");

        double startingAngle = bot.readFile(bot.GYRO_ANGLE_FILE_NAME);

        bot.setAngleZzeroValue(-startingAngle);
        //bot.setPoseEstimate(new Pose2d(-36,-63,startingAngle));

        //get the angle the robot was at when auton ended
        //bot.setAngleZzeroValue(-bot.readFile(bot.GYRO_ANGLE_FILE_NAME));
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, 0); //reset the old angle to zero

        telemetry.log().add("Ready to start");
        telemetry.log().add("gyro angle: " + bot.getIMUAngle());
        telemetry.log().add("lift max: " + bot.liftMax);

        bot.setBulkReadAuto();

        waitForStart();

        slamra.start();

        while (opModeIsActive()) {

            //grabberToggle = new ToggleDouble(new double[] {bot.grabberMin, bot.grabberMax}, grabberToggle.getToggleIndex());
//            fondationGrabberPos.changeValue(FoundationGrabberMax,1);
//            fondationGrabberPos.changeValue(FoundationGrabberMin,0);
//            frontGrabberPos.changeValue(FrontGrabberMin,0);
//            frontGrabberPos.changeValue(FrontGrabberMax,1);
//            backGrabberPos.changeValue(BackGrabberMin,0);
//            backGrabberPos.changeValue(BackGrabberMax,1);

            bot.updateGamepads();

            /*
            Speed Modifiers
             */
            if (bot.x1.onPress()) speedModifier =.60;
            if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifier =.9;
            if (bot.a1.onPress() && !bot.start1.isPressed()) speedModifier =1.7; //trig math caps speed at .7, 1.3 balances it out

            if (bot.y1.onPress()) fieldRelative.toggle(); //toggle field relative

            if (fieldRelative.output()) //if field relative is enabled
            {
                double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
                double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //get angle
                double robotAngle = Math.toRadians(bot.getIMUAngle()); //angle of robot
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

            //set motor powers based on previous calculations
            bot.setMotorPowers(leftFrontPower* speedModifier, leftBackPower* speedModifier, rightBackPower* speedModifier, rightFrontPower* speedModifier);

            /*
            Toggle Intake:
            Triggers control intake/outtake
            Push the same trigger twice to turn the intake motors off
             */
            if (bot.rightTrigger2.onPress()) //intake
            {
                if (!intake)
                {
                    intake = true;
                    outtake = false;
                    if (!bot.isStoneAtEnd()) {
                        bot.intake(1);
                    }
                    else bot.intake(.5);
                }
                else //intake button pressed again
                {
                    intake = false;
                    outtake = false;
                    bot.stopIntake();
                }
            }
            else if (bot.leftTrigger2.onPress()) //outtake
            {
                if (!outtake) {
                    intake = false;
                    outtake = true;
                    bot.outtake(1);
                }
                else //outtake button pressed again
                {
                    intake = false;
                    outtake = false;
                    bot.stopIntake();
                }
            }

            if (bot.a2.onPress()) //toggle fondation grabber
            {
                fondationGrabberPos.toggle();
                bot.setFondationGrabber(fondationGrabberPos.output());
            }

            if (bot.b2.onPress()) //toggle stone grabber
            {
                grabberToggle.toggle();
                bot.setGrabber(grabberToggle.output());
            }
            if (bot.x2.onPress())
            {
                bot.setGrabber(2);
            }
            if (bot.y2.isPressed())
            {
                bot.extendMeasuringTape();
            }
            else if (bot.dpadUp2.isPressed())
            {
                bot.retractMeasuringTape();
            }
            else
            {
                bot.stopMeasuringTape();
            }

            double liftSpeed = -gamepad2.left_stick_y;
            /*
            Lift controls
            Deadzone of .05
            has a maximum, but no minimum
             */
            if (liftSpeed > .05 || liftSpeed < -.05) //if lift is below max and speed is outside of deadzone
            {
                bot.setLiftPower(liftSpeed);
            }
            else
            {
                bot.setLiftPower(0);
            }

            //old way to move servo
            //if (bot.rightBumper2.isPressed()) bot.updateServo(bot.grabberSlide, 1, slideSpeed, bot.grabberMax, bot.grabberMin); //move horizontal slide back
            //if (bot.leftBumper2.isPressed()) bot.updateServo(bot.grabberSlide, -1, slideSpeed, bot.grabberMax, bot.grabberMin); //move horizontal slide forward

            if (bot.rightBumper2.isPressed()) bot.ExtendGrabberSlide();
            else if (bot.leftBumper2.isPressed()) bot.RetractGrabberSlide();
            else bot.stopGrabberSlide();

            if (gamepad1.right_bumper && gamepad1.left_bumper) //calibrate gyro
            {
                double zeroVal = -Math.toDegrees(bot.getRawExternalHeading());
                bot.setAngleZzeroValue(zeroVal);
                telemetry.log().add("Calibrated, set zero value to" + zeroVal);
            }

            final int robotRadius = 9; // inches

            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if (up == null) RobotLog.v("Realsense NULL");
            else {

                // We divide by 0.0254 to convert meters to inches
                Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
                Rotation2d rotation = up.pose.getRotation();

                field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
                double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
                double x1 = translation.getX() + arrowX / 2, y1 = translation.getY() + arrowY / 2;
                double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
                field.strokeLine(x1, y1, x2, y2);

                packet.addLine("Confidence: " + up.confidence);
                packet.addLine("Translation X:" + translation.getX() + ", Y:" + translation.getY());
                packet.addLine("Get Rotation Degrees " + up.pose.getRotation().getDegrees());
                packet.addLine("Get Rotation Heading " + up.pose.getHeading());


                dashboard.sendTelemetryPacket(packet);
            }
            /*
            Telemetry
             */
            //Pose2d pos = bot.getPoseEstimate();
            //bot.setPoseEstimate(new Pose2d(pos.getX(), pos.getY(), bot.getRawExternalHeading()+Math.toRadians(90)));
            //bot.updateRobotPosRoadRunner(); //display robot's position
//            telemetry.addData("LEFT FRONT AMPS:", bot.leftFront.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("RIGHT FRONT AMPS",bot.rightFront.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("LEFT BACK AMPS:", bot.leftRear.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("RIGHT BACK AMPS",bot.rightRear.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("LIFT AMPS",bot.lift.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("INTAKE AMPS",bot.intake.getCurrent(CurrentUnit.AMPS));
            //telemetry.update();
        }
        slamra.stop();
        bot.stopRobot();
    }
}
