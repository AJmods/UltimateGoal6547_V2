package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DriveSpeeds;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.homar.ToggleBoolean;
import org.firstinspires.ftc.teamcode.util.homar.ToggleInt;
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.ThrowerUtil;

/*
This is the tele-op we use to drive the robot
 */
@Config
@TeleOp(name = "Meet 3 Tele-op", group = "_teleOp")
public class Meet3Teleop extends LinearOpMode {

    public static boolean USE_CALCULATED_VELOCITY = false;
    public static double REV_PER_SEC = 44;
    public static double TARGET_HIGHET = FieldConstants.RED_GOAL_HEIGHT;

    private final double RED_GOAL_X = FieldConstants.RED_GOAL_X;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    private ToggleBoolean feildRealtive = new ToggleBoolean(true);
    ToggleBoolean grab = new ToggleBoolean(false);
    ToggleBoolean lowerWobwater = new ToggleBoolean(false);
    ToggleInt powerShot = new ToggleInt(3);

    private DriveTrain6547Realsense bot; //the robot class

    private double angleZeroValue = 0;

    private double robotAngle=0;

    public static double launcherDistanceFromRealsense = 0;
    public static double launcherXDistanceFromRealsense = 0;

    boolean isIntaking = false;
    boolean isOuttaking = false;

    @Override
    public void runOpMode() throws InterruptedException{
       // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //makes telemetry output to the FTC Dashboard
        bot = new DriveTrain6547Realsense(this, true);
        telemetry.log().add("Initing Vuforia");
        bot.initVufoira();
        telemetry.update();
        try {
            Pose2d currentPos = bot.readPos();
            bot.setPoseEstimate(currentPos);
            RobotLog.v("Read pos as " + currentPos.toString());
            angleZeroValue = Math.toRadians(90); //currentPos.getHeading();
            telemetry.log().add("set POS to " + currentPos.toString());
            RobotLog.v("set POS to " + currentPos.toString());
        } catch (Exception e) {telemetry.log().add("FAILED TO READ POSITION");}
        telemetry.log().add("DID POS THING");
        //reset POS to 0
        bot.savePos(new Pose2d(0,0,0));

        telemetry.log().add("Ready to start");

        waitForStart();

        bot.startVuforia();

        telemetry.log().add("CONTROLS:");
        telemetry.log().add("Gamepad1: TOGGLE field relative: Y");
        telemetry.log().add("Gamepad1: CALABRATE feild relative: l and r bumpers");
        telemetry.log().add("Gamepad1: ROBOT SPEED modifers: X, B, A");
        telemetry.log().add("Gamepad2: INTAKE: Left and Right Triggers");
        telemetry.log().add("Gamepad2: INDEXER: A button");
        telemetry.log().add("Gamepad2: TURN ON/OFF THROWER: LB");

        double speedModifer=1;

        ToggleBoolean TurnOnThrower = new ToggleBoolean(false);

        //T265LocalizerRR.slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(1,1), new Rotation2d(1.57)));



        while (opModeIsActive()) {

            Pose2d pos = bot.getPoseEstimate();

            bot.updateGamepads();
            /*
            Speed Modifers
             */
            if (bot.mode == DriveTrain6547Realsense.Mode.IDLE) {
                if (bot.x1.onPress()) speedModifer = .60;
                if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifer = .9;
                if (bot.a1.onPress() && !bot.start1.isPressed())
                    speedModifer = 1.3; //trig math caps speed at .7, 1.3 balences it out

                if (bot.y1.onPress()) feildRealtive.toggle(); //toggle field realtive

                robotAngle = bot.getRawExternalHeading() - angleZeroValue; //angle of robot

                if (feildRealtive.output()) //if field relative is enabled
                {
                    double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
                    double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //get angle
                    double rightX = gamepad1.right_stick_x * 2; //rotation
                    rightX *= .5; //half rotation value for better turning
                    //offset the angle by the angle of the robot to make it field realtive
                    leftFrontPower = speed * Math.cos(LeftStickAngle - robotAngle) + rightX;
                    rightFrontPower = speed * Math.sin(LeftStickAngle - robotAngle) - rightX;
                    leftBackPower = speed * Math.sin(LeftStickAngle - robotAngle) + rightX;
                    rightBackPower = speed * Math.cos(LeftStickAngle - robotAngle) - rightX;

                    telemetry.addData("LS angle", Math.toDegrees(LeftStickAngle));
                    telemetry.addData("driving toward", LeftStickAngle - robotAngle);
                    telemetry.addData("ROBOT ANGLE", Math.toDegrees(robotAngle));
                    telemetry.addData("RAW ANGLE", Math.toDegrees(bot.getRawExternalHeading()));
                } else //regular drive (different math because this is faster than sins and cosines
                {
                    leftFrontPower = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                    rightFrontPower = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
                    leftBackPower = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                    rightBackPower = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
                }
                leftFrontPower *= speedModifer;
                leftBackPower *= speedModifer;
                rightBackPower *= speedModifer;
                rightFrontPower *= speedModifer;

                telemetry.addData("leftFront Power", leftFrontPower);

                bot.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

                if (bot.rightBumper1.onPress() && bot.leftBumper1.onPress()) //calibrate gyro
                {
                    //double zeroVal = -Math.toDegrees(bot.getRawExternalHeading());
                    angleZeroValue = bot.getRawExternalHeading();
                    telemetry.log().add("Calibrated, set zero value to " + angleZeroValue);
                }
                //interrupt all robot actions to turn toward goal.
//            if (gamepad1.left_stick_button) {
//               // bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), bot.getPoseEstimate()));
//                bot.turnRealtiveSync(Math.toRadians(0));
//            }
                if (gamepad1.left_stick_button) {
                    // bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), bot.getPoseEstimate()));
                    bot.turnRealtiveSync(Math.toRadians(0));
                }

            }

            if (gamepad1.right_stick_button && bot.mode == DriveTrain6547Realsense.Mode.IDLE) {
                bot.followTrajectory(bot.trajectoryBuilder(false, DriveSpeeds.fast).lineToLinearHeading(new Pose2d(0,-36, Math.toRadians(0))).build());
            }

            if (Math.abs(gamepad1.left_stick_x) > .3 || Math.abs(gamepad1.left_stick_y) > .3 || Math.abs(gamepad1.right_stick_x) > .3 || Math.abs(gamepad1.right_stick_y) > .3) {
                bot.mode = DriveTrain6547Realsense.Mode.IDLE;
            }

            if (gamepad2.a) {
                bot.launchRing();
                RobotLog.v("Thrower motor 0 VELO when lauched: " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
            } else bot.openIndexer();

            if (bot.b2.onPress()) grab.toggle();
            if (bot.y2.onPress()) lowerWobwater.toggle();

            if (grab.output()) {
                bot.grabWobbleGoal();
            } else bot.releaseWobbleGoal();

            if (lowerWobwater.output()) {
                bot.lowerWobvator();
            } else bot.raiseWobvator();

            if (bot.leftTrigger2.onPress() && !isIntaking) {
                bot.intake();
                isIntaking = true;
                isOuttaking = false;
            } else if (bot.leftTrigger2.onPress() && isIntaking) {
                bot.stopIntake();
                isIntaking = false;
                isOuttaking = false;
            }

            if (bot.rightTrigger2.onPress() && !isOuttaking) {
                bot.outtake();
                isOuttaking = true;
                isIntaking = false;
            } else if (bot.rightTrigger2.onPress() && isOuttaking) {
                bot.stopIntake();
                isIntaking = false;
                isOuttaking = false;
            }

           // if (bot.rightBumper2.onPress()) bot.setPoseEstimate(new Pose2d(1,1,Math.toRadians(90)));

//            if (gamepad2.left_bumper && gamepad2.right_bumper) {
//                powerShot.toggle();
//                if (powerShot.output() == 0) {
//                    bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), bot.getPoseEstimate()));
//                } else if (powerShot.output() == 1) {
//
//                } else if (powerShot.output() == 2) {
//
//                }
//                //throw ring
//                setThrowerToTarget(bot.getPoseEstimate());
//                sleep(500);
//                bot.launchRing();
//                sleep(500);
//                bot.openIndexer();
//            }

            //do thrower calculations

            boolean isValidAngle = ThrowerUtil.isValidAngle(pos.getX(), pos.getY(), pos.getHeading());

            double targetY = ThrowerUtil.getTargetY(pos, RED_GOAL_X);
            double deltaX = pos.getX() - RED_GOAL_X;
            double detlaY = pos.getY() - targetY;
            double dist = Math.hypot(deltaX, detlaY);
            double vi = ThrowerUtil.getVi(0, ThrowerUtil.INITAL_HEIGHT, dist + launcherDistanceFromRealsense, TARGET_HIGHET, ThrowerUtil.INITAL_ANGLE);
            double targetRev = vi/ ThrowerUtil.inchesPerRev;

            if (bot.leftBumper2.onPress()) {
                TurnOnThrower.toggle();
            }

            if (TurnOnThrower.output() && !USE_CALCULATED_VELOCITY) {
                bot.setThrowerVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);
            } else if (TurnOnThrower.output() && USE_CALCULATED_VELOCITY) {
                bot.setThrowerVelocity(bot.getThrowerVeloctiyFromPosition(pos, AngleUnit.DEGREES), AngleUnit.DEGREES);
            } else {
                bot.thrower1.setPower(0);
                bot.thrower2.setPower(0);
            }

//            if (USE_CALCULATED_VELOCITY) {
//                bot.setThrowerVelocity(targetRev*360, AngleUnit.DEGREES);
//            } else {
//                bot.setThrowerVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);
//            }
            //stroke launcher targetPos

            if (gamepad1.left_stick_button) {
                bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), bot.getPoseEstimate()));
            }

            bot.setPacketAction((packet, fieldOverlay) -> {

                if (!USE_CALCULATED_VELOCITY) {
                   packet.addLine("USING USER-INPUTED VELOCITY");
                } else {
                    packet.addLine("USING CALCULATED VELOCITY");
                }

                double drawsConstant = REV_PER_SEC/targetRev;
                packet.addLine("Current Motor Rev/s)/(Target Rev/s): " + drawsConstant + " (Danda's Constant)");
                if (isValidAngle) {
                    packet.addLine("VALID ANGLE");
                } else {
                    double minAngle = Math.toDegrees(Math.atan2(ThrowerUtil.MIN_Y - pos.getY(), RED_GOAL_X - pos.getX()));
                    double maxAngle = Math.toDegrees(Math.atan2(ThrowerUtil.MAX_Y - pos.getY(), RED_GOAL_X - pos.getX()));
                    packet.addLine("INVALID ANGLE! Must be Between " + Math.round(minAngle) + " and " + Math.round(maxAngle) + " Degrees.");
                }
                //add data.
                double[] throwerVelocities = bot.getThrowerVelocity(AngleUnit.DEGREES);
                packet.addLine("Target Ring Velocity: " + vi + " inches/s, " + targetRev + " rev/s, " + (vi*28) + " ticks/s");
                try {
                    packet.addLine("Thrower1 velocity: " + (throwerVelocities[0] / 360 * ThrowerUtil.inchesPerRev) + " inches/s, " + (throwerVelocities[0] / 360) + " rev/s, " + bot.thrower1.getVelocity() + " ticks/s");
                    packet.addLine("Thrower2 velocity: " + (throwerVelocities[1] / 360 * ThrowerUtil.inchesPerRev) + " inches/s, " + (throwerVelocities[1] / 360) + " rev/s, " + bot.thrower2.getVelocity() + " ticks/s");
                } catch (Exception ignored) {}

                //draw robot
                DashboardUtil.drawRobot(fieldOverlay, pos);
                //draw launcherPos
                double distX = launcherDistanceFromRealsense * Math.cos(pos.getHeading());
                double distY = launcherDistanceFromRealsense * Math.sin(pos.getHeading());

                double distX2 = launcherXDistanceFromRealsense * Math.sin(pos.getHeading());
                double distY2 = launcherXDistanceFromRealsense * Math.cos(pos.getHeading());
                fieldOverlay.strokeCircle(pos.getX() - distX - distX2, pos.getY() - distY-distY2, 3);
                //draw line in the direction of where the robot is facing
                //DashboardUtil.drawLine(fieldOverlay, pos);

                //draw target robot angle range
                fieldOverlay.setStroke("#000000");
                fieldOverlay.strokeLine(pos.getX(), pos.getY(), RED_GOAL_X, ThrowerUtil.MIN_Y);
                fieldOverlay.strokeLine(pos.getX(), pos.getY(), RED_GOAL_X, ThrowerUtil.MAX_Y);

                //mark middle launch goal with circle
                fieldOverlay.strokeCircle(RED_GOAL_X, targetY, 1);

                //draw robot launch goal position
                fieldOverlay.setStrokeWidth(3);
                fieldOverlay.setStroke("FF0000");
                fieldOverlay.strokeLine(RED_GOAL_X, ThrowerUtil.MIN_Y, RED_GOAL_X, ThrowerUtil.MAX_Y);

            });

            telemetry.addData("Target VELO: ", REV_PER_SEC);
            telemetry.addData("CURRENT THROWER 0 VELO:", bot.getThrowerVelocity(AngleUnit.DEGREES)[0]/360);
            telemetry.addData("CURRENT THROWER 1 VELO: ", bot.getThrowerVelocity(AngleUnit.DEGREES)[1]/360);
            telemetry.addData("IS LAUNCHED: ", bot.isReadyToThrow());
            telemetry.update();
            bot.update(); //updates robot's position
            bot.updateLightsBasedOnThrower();

        }
        bot.stopVuforia();
        bot.stopRobot();
    }

    public void setThrowerToTarget(Vector2d startPos, double angle) {
        setThrowerToTarget(startPos, angle, true);
    }
    public void setThrowerToTarget(Vector2d startPos, double angle, boolean isPowerShot) {
        setThrowerToTarget(new Pose2d(startPos.getX(), startPos.getY(), angle), isPowerShot);
    }
    public void setThrowerToTarget(Pose2d startPos) {
        setThrowerToTarget(startPos, true);
    }
    public void setThrowerToTarget(Pose2d startPos, boolean isPowerShot) {
        double targetY = ThrowerUtil.getTargetY(startPos, FieldConstants.TOP_OF_FIELD);
        double dist = Math.hypot(startPos.getX() - FieldConstants.TOP_OF_FIELD, startPos.getY() - targetY);
        if (isPowerShot) {
            double vi = ThrowerUtil.getVi(startPos.getX(), ThrowerUtil.INITAL_HEIGHT, dist, FieldConstants.POWER_SHOT_HEIGHT, ThrowerUtil.INITAL_ANGLE);
            double targetRevPerSec = vi / ThrowerUtil.inchesPerRev;
            bot.setThrowerVelocity(targetRevPerSec * 360 * ThrowerUtil.POWER_SHOT_CONSTANT, AngleUnit.DEGREES);
            RobotLog.v("Set speed to " + (targetRevPerSec*360) + "Rev/s");
        }
        else {
            double vi = ThrowerUtil.getVi(startPos.getX(), ThrowerUtil.INITAL_HEIGHT, dist, FieldConstants.RED_GOAL_HEIGHT, ThrowerUtil.INITAL_ANGLE);
            double targetRevPerSec = vi / ThrowerUtil.inchesPerRev;
            bot.setThrowerVelocity(targetRevPerSec * 360, AngleUnit.DEGREES);
            RobotLog.v("Set speed to " + (targetRevPerSec*360) + "Rev/s");
        }
    }


}
