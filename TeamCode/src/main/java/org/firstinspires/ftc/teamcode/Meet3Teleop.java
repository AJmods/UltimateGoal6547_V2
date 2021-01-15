package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

/**
 * This is the main tele-op the drivers use
 * @see DriveTrain6547Realsense
 */
@Config
@TeleOp(name = "Meet 3 Tele-op", group = "_teleOp")
public class Meet3Teleop extends LinearOpMode {

    public static boolean USE_CALCULATED_VELOCITY = false;
    public static double REV_PER_SEC = 48;
    public static double VELO_MUTIPLIER = 1;
    public static double TARGET_HEIGHT = FieldConstants.RED_GOAL_HEIGHT;

    private final double RED_GOAL_X = FieldConstants.RED_GOAL_X;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    private final ToggleBoolean fieldRelative = new ToggleBoolean(true);
    ToggleBoolean grab = new ToggleBoolean(false);
    ToggleBoolean lowerWobvator = new ToggleBoolean(false);
    ToggleInt powerShot = new ToggleInt(3);

    private DriveTrain6547Realsense bot; //the robot class

    private double angleZeroValue = 0;

    private double robotAngle=0;

    public static double launcherDistanceFromRealsense = 0;
    public static double launcherXDistanceFromRealsense = 0;

    boolean isIntaking = false;
    boolean isOuttaking = false;

    boolean messageDisplayed = false;

    @Override
    public void runOpMode() throws InterruptedException{
       // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //makes telemetry output to the FTC Dashboard
        bot = new DriveTrain6547Realsense(this, false);
        telemetry.log().add("Initing Vuforia");
        bot.initVufoira();
        bot.update();
        telemetry.update();
        try {
            Pose2d currentPos = bot.readPos();
            bot.setPoseEstimate(currentPos);
            RobotLog.v("Read pos as " + currentPos.toString());
            angleZeroValue = Math.toRadians(90); //currentPos.getHeading();
            telemetry.log().add("set POS to " + currentPos.toString());
            RobotLog.v("set POS to " + currentPos.toString());
        } catch (Exception e) {telemetry.log().add("FAILED TO READ POSITION");}
        //reset POS to 0
        bot.savePos(new Pose2d(0,0,0));

        telemetry.log().add("Ready to start");

        waitForStart();

        bot.startVuforia();

        telemetry.log().add("CONTROLS:");
        telemetry.log().add("Gamepad1: TOGGLE field relative: Y");
        telemetry.log().add("Gamepad1: CALIBRATE field relative: l and r bumpers");
        telemetry.log().add("Gamepad1: ROBOT SPEED modifiers: X, B, A");
        telemetry.log().add("Gamepad2: INTAKE: Left and Right Triggers");
        telemetry.log().add("Gamepad2: INDEXER: A button");
        telemetry.log().add("Gamepad2: TURN ON/OFF THROWER: LB");

        double speedModifier=1;

        ToggleBoolean TurnOnThrower = new ToggleBoolean(false);

        //T265LocalizerRR.slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(1,1), new Rotation2d(1.57)));



        while (opModeIsActive()) {

            Pose2d pos = bot.getPoseEstimate();

            bot.updateGamepads();
            /*
            Speed Modifiers
             */
            if (bot.mode == DriveTrain6547Realsense.Mode.IDLE) {
                if (bot.x1.onPress()) speedModifier = .60;
                if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifier = .9;
                if (bot.a1.onPress() && !bot.start1.isPressed())
                    speedModifier = 1.3; //trig math caps speed at .7, 1.3 balances it out

                if (bot.y1.onPress()) fieldRelative.toggle(); //toggle field relative

                robotAngle = bot.getRawExternalHeading() - angleZeroValue; //angle of robot

                if (fieldRelative.output()) //if field relative is enabled
                {
                    double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
                    double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //get angle
                    double rightX = gamepad1.right_stick_x * 2; //rotation
                    rightX *= .5; //half rotation value for better turning
                    //offset the angle by the angle of the robot to make it field relative
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
                leftFrontPower *= speedModifier;
                leftBackPower *= speedModifier;
                rightBackPower *= speedModifier;
                rightFrontPower *= speedModifier;

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
//               // bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), bot.getPoseEstimate()));
//                bot.turnRelativeSync(Math.toRadians(0));
//            }
                if (gamepad1.left_stick_button) {
                    // bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), bot.getPoseEstimate()));
                    bot.turnRelativeSync(Math.toRadians(0));
                }

            }

            if (gamepad1.right_stick_button && bot.mode == DriveTrain6547Realsense.Mode.IDLE) {
                bot.followTrajectory(bot.trajectoryBuilder(false, DriveSpeeds.fast).lineToLinearHeading(new Pose2d(0,-39, Math.toRadians(0))).build());
            }

            if (bot.dpadUp1.onPress()) {
                doPowerShots(pos);
                bot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
            if (bot.dpadDown1.onPress()) {
                doRegularShots(pos);
                bot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            if (isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y) || isStickMoved(gamepad1.right_stick_x, gamepad1.right_stick_y)) {
                bot.mode = DriveTrain6547Realsense.Mode.IDLE;
            }

            if (gamepad2.a) {
                bot.launchRing();
                RobotLog.v("Thrower motor 0 VELO when launched: " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
            } else bot.openIndexer();

            if (bot.b2.onPress()) grab.toggle();
            if (bot.y2.onPress()) lowerWobvator.toggle();

            if (grab.output()) {
                bot.grabWobbleGoal();
            } else bot.releaseWobbleGoal();

            if (lowerWobvator.output()) {
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

            if (bot.dpadUp2.onPress()) {
                bot.stopIntake();
            }

           // if (bot.rightBumper2.onPress()) bot.setPoseEstimate(new Pose2d(1,1,Math.toRadians(90)));

//            if (gamepad2.left_bumper && gamepad2.right_bumper) {
//                powerShot.toggle();
//                if (powerShot.output() == 0) {
//                    bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), bot.getPoseEstimate()));
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
            double deltaY = pos.getY() - targetY;
            double dist = Math.hypot(deltaX, deltaY);
            double vi = ThrowerUtil.getVi(0, ThrowerUtil.INITIAL_HEIGHT, dist + launcherDistanceFromRealsense, TARGET_HEIGHT, ThrowerUtil.INITIAL_ANGLE);
            double targetRev = vi/ ThrowerUtil.inchesPerRev;

            if (bot.leftBumper2.onPress()) {
                TurnOnThrower.toggle();
            }

            if (TurnOnThrower.output() && !USE_CALCULATED_VELOCITY) {
                bot.setThrowerVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);
                bot.updateLightsBasedOnThrower();
            } else if (TurnOnThrower.output() && USE_CALCULATED_VELOCITY) {
                bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(pos, AngleUnit.DEGREES)*VELO_MUTIPLIER, AngleUnit.DEGREES);
                bot.updateLightsBasedOnThrower();
            } else {
                bot.thrower1.setPower(0);
                bot.thrower2.setPower(0);
                bot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

//            if (USE_CALCULATED_VELOCITY) {
//                bot.setThrowerVelocity(targetRev*360, AngleUnit.DEGREES);
//            } else {
//                bot.setThrowerVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);
//            }
            //stroke launcher targetPos

//            if (gamepad1.left_stick_button) {
//                bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), bot.getPoseEstimate()));
//            }

            if (!bot.isInsideField(pos) && !messageDisplayed) {
                RobotLog.setGlobalWarningMessage("POSITION ERROR, SCAN VUFORIA TARGET");
                messageDisplayed = true;
            } else if (bot.isInsideField(pos) && messageDisplayed) {
                RobotLog.clearGlobalWarningMsg();
                messageDisplayed = false;
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
            //bot.updateLightsBasedOnThrower();
            //if (bot.isInsideField(pos)) RobotLog.setGlobalWarningMessage("ROBOT POSE OFF, GO TO VUFORIA TARGET");

        }
        bot.stopVuforia();
        bot.stopRobot();
    }
    public boolean isStickMoved(double x, double y) {
        return Math.abs(x) > .3 || Math.abs(y) > .3;
    }

    public void doRegularShots(Pose2d currentPos) {
        bot.stopIntake();
        double angleToTurnTo = bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), currentPos);
        bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(new Pose2d(currentPos.getX(), currentPos.getY(), angleToTurnTo), AngleUnit.DEGREES), AngleUnit.DEGREES);

        bot.turnRelativeSync(angleToTurnTo);
        bot.turnRelativeSync(angleToTurnTo);
        //bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(currentPos, AngleUnit.DEGREES) * VELO_MUTIPLIER, AngleUnit.DEGREES);

        for (int i = 0; i < 3 && opModeIsActive() && !isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y); i++) {
            while (!bot.isReadyToThrow() && opModeIsActive() && !isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y)) {bot.updateLightsBasedOnThrower();}
            bot.launchRing();
            RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
            if (!isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y)) sleep(500);
            bot.openIndexer();
            if (!isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y)) sleep(500);
        }
    }

    public void doPowerShots(Pose2d currentPos) {
        bot.stopIntake();
       // Vector2d launchPos = new Vector2d(0,-14);
        //setThrowerToTarget(new Vector2d(currentPos.getX(), currentPos.getY()), Math.toRadians(0));

        //drive to middle, and face first power shot goal, prepare to launch.
//        bot.followTrajectory(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(launchPos.getX(), launchPos.getY(), Math.toRadians(0))).build());
//
//        while (!isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y) && opModeIsActive()) { bot.update();}
        setThrowerToTarget(bot.getPoseEstimate());

        //telemetry.log().add("Drive to PowerShot");

        //turn toward 3 power shots
        RobotLog.v("Launching Power Shot 1");
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y), bot.getPoseEstimate()));
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y), bot.getPoseEstimate()));
        //throw ring
        setThrowerToTarget(bot.getPoseEstimate());
        //wait for launch speed to be ready
        while (!bot.isReadyToThrow()) {bot.updateLightsBasedOnThrower();}
        bot.launchRing();
        RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        sleep(500);
        bot.openIndexer();
        //prepare to throw next ring
        RobotLog.v("Launching Power Shot 2");
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y), bot.getPoseEstimate()));
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y), bot.getPoseEstimate()));
        //throw ring
        setThrowerToTarget(bot.getPoseEstimate());
        //wait for launch speed to be ready
        while (!bot.isReadyToThrow()) {bot.updateLightsBasedOnThrower();}
        bot.launchRing();
        RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        sleep(500);
        bot.openIndexer();
        //prepare to throw next ring
        RobotLog.v("Launching Power Shot 3");
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), bot.getPoseEstimate()));
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), bot.getPoseEstimate()));
        //throw ring
        setThrowerToTarget(bot.getPoseEstimate());
        //wait for launch speed to be ready
        while (!bot.isReadyToThrow()) {bot.updateLightsBasedOnThrower();}
        bot.launchRing();
        RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        sleep(500);
        bot.openIndexer();
        //stop thrower
        bot.setThrowerVelocity(0);

        telemetry.log().add("Launched Rings");
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
            double vi = ThrowerUtil.getVi(startPos.getX(), ThrowerUtil.INITIAL_HEIGHT, dist, FieldConstants.POWER_SHOT_HEIGHT, ThrowerUtil.INITIAL_ANGLE);
            double targetRevPerSec = vi / ThrowerUtil.inchesPerRev;
            bot.setThrowerVelocity(targetRevPerSec * 360 * ThrowerUtil.POWER_SHOT_CONSTANT, AngleUnit.DEGREES);
            RobotLog.v("Set speed to " + (targetRevPerSec*360) + "Rev/s");
        }
        else {
            double vi = ThrowerUtil.getVi(startPos.getX(), ThrowerUtil.INITIAL_HEIGHT, dist, FieldConstants.RED_GOAL_HEIGHT, ThrowerUtil.INITIAL_ANGLE);
            double targetRevPerSec = vi / ThrowerUtil.inchesPerRev;
            bot.setThrowerVelocity(targetRevPerSec * 360, AngleUnit.DEGREES);
            RobotLog.v("Set speed to " + (targetRevPerSec*360) + "Rev/s");
        }
    }


}
