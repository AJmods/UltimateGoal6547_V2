package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DriveSpeeds;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.drivetrain.localizer.T265LocalizerRR;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.ThrowerUtil;
import org.firstinspires.ftc.teamcode.util.homar.ToggleBoolean;
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil;

/**
 * This is the main tele-op the drivers use
 * @see DriveTrain6547Realsense
 */
@Config
@TeleOp(name = "League Championship Tele-op NEW", group = "_teleOp")
@Disabled
public class LeagueChampionshipTeleopNew extends LinearOpMode {

    public static double RED_POWERSHOT_X1 = FieldConstants.RED_POWER_SHOT_1X;
    public static double RED_POWERSHOT_Y1 = FieldConstants.RED_POWER_SHOT_1Y;
    public static double RED_POWERSHOT_X2 = FieldConstants.RED_POWER_SHOT_2X;
    public static double RED_POWERSHOT_Y2 = FieldConstants.RED_POWER_SHOT_2Y;
    public static double RED_POWERSHOT_X3 = FieldConstants.RED_POWER_SHOT_3X;
    public static double RED_POWERSHOT_Y3 = FieldConstants.RED_POWER_SHOT_3Y;

    public static double PowerShot1Modifer = 1;
    public static double PowerShot2Modifer = 1;
    public static double PowerShot3Modifer = 1;

    public static boolean USE_CALCULATED_VELOCITY = false;
    public static double REV_PER_SEC = 48;
    public static double VELO_MUTIPLIER = 1;
    public static double TARGET_HEIGHT = FieldConstants.RED_GOAL_HEIGHT;

    public static double ANGLE_MODIFER = 0;

    private final double RED_GOAL_X = FieldConstants.RED_GOAL_X;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    private final ToggleBoolean fieldRelative = new ToggleBoolean(true);
    ToggleBoolean grab = new ToggleBoolean(false);
    ToggleBoolean lowerWobvator = new ToggleBoolean(false);
    //ToggleInt powerShot = new ToggleInt(3);

    private DriveTrain6547Realsense bot; //the robot class

    private double angleZeroValue = 90;

    private double robotAngle=0;

    public static double launcherDistanceFromRealsense = 0;
    public static double launcherXDistanceFromRealsense = 0;

    boolean isIntaking = false;
    boolean isOuttaking = false;

    boolean messageDisplayed = false;

    double speedModifier=1;

    ToggleBoolean TurnOnThrower = new ToggleBoolean(false);

    OpMode opMode;

    public LeagueChampionshipTeleopNew(OpMode opMode, DriveTrain6547Realsense bot) {
        this.opMode=opMode;
        this.bot=bot;
    }

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.log().add("Initing Tele-op");
        RobotLog.v("Initing Tele-op");
        // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //makes telemetry output to the FTC Dashboard
        bot = new DriveTrain6547Realsense(this, false);
        RobotLog.v("Inited Robot");
        LeagueChampionshipTeleopNew leagueChampionshipTeleop = new LeagueChampionshipTeleopNew(this, bot);

        DriveTrain6547Realsense.INTERRUPT_TRAJECTORIES_WITH_GAMEPAD = true;
        telemetry.log().add("Initing Vuforia (takes some time)");
        bot.initVufoira();
        bot.update();
        telemetry.update();
        try {
            Pose2d currentPos = bot.readPos();
            bot.setPoseEstimate(currentPos);
            RobotLog.v("Read pos as " + currentPos.toString());
            //angleZeroValue = currentPos.getHeading(); //currentPos.getHeading();
            telemetry.log().add("set POS to " + currentPos.toString());
            RobotLog.v("set POS to " + currentPos.toString());
        } catch (Exception e) {telemetry.log().add("FAILED TO READ POSITION");}
        //reset POS to 0
        bot.savePos(new Pose2d(0,0,Math.toRadians(0)));

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

        //T265LocalizerRR.slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(1,1), new Rotation2d(1.57)));



        while (opModeIsActive()) {
            leagueChampionshipTeleop.doTeleOp();

            if (bot.intake.getCurrentAlert(CurrentUnit.AMPS) > 7) {
                //RobotLog.setGlobalWarningMessage("INTAKE JAMMED");
            }
            Pose2d pos = bot.getPoseEstimate();
            if (!bot.isInsideField(pos) && !messageDisplayed) {
                RobotLog.setGlobalWarningMessage("POSITION ERROR, SCAN VUFORIA TARGET");
                messageDisplayed = true;
            } else if (bot.isInsideField(pos) && messageDisplayed) {
                RobotLog.clearGlobalWarningMsg();
                messageDisplayed = false;
            }

            T265Camera.PoseConfidence confidence = T265LocalizerRR.getConfidence();

            boolean isValidAngle = ThrowerUtil.isValidAngle(pos.getX(), pos.getY(), pos.getHeading());

            double targetY = ThrowerUtil.getTargetY(pos, RED_GOAL_X);
            double deltaX = pos.getX() - RED_GOAL_X;
            double deltaY = pos.getY() - targetY;
            double dist = Math.hypot(deltaX, deltaY);
            double vi = ThrowerUtil.getVi(0, ThrowerUtil.INITIAL_HEIGHT, dist + launcherDistanceFromRealsense, TARGET_HEIGHT, ThrowerUtil.INITIAL_ANGLE);
            double targetRev = vi/ ThrowerUtil.inchesPerRev;

            bot.setPacketAction((packet, fieldOverlay) -> {

                if (!USE_CALCULATED_VELOCITY) {
                    packet.addLine("USING USER-INPUTED VELOCITY");
                } else {
                    packet.addLine("USING CALCULATED VELOCITY");
                }

                packet.addLine("Realsense Confidence: " + confidence.name());

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

//                packet.addLine("distance Voltage: " + bot.distanceSensorX.getVoltage());
//                packet.addLine("Distance Inches (Guess) " + bot.distanceSensorY.getVoltage()*492.126/2.54);

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

                fieldOverlay.setStroke("#0069fc");
                fieldOverlay.strokeCircle(RED_POWERSHOT_X1, RED_POWERSHOT_Y1,3);
                fieldOverlay.strokeCircle(RED_POWERSHOT_X2, RED_POWERSHOT_Y2,3);
                fieldOverlay.strokeCircle(RED_POWERSHOT_X3, RED_POWERSHOT_Y3,3);

            });

            telemetry.addData("REALSENSE ANGLE (deg) ", Math.toDegrees(bot.getRawExternalHeading()));
            telemetry.addData("IMU ANGLE (deg)", Math.toDegrees(bot.getRawIMUangle()));
            telemetry.addData("ANALOG GYRO ANGLE (deg) ", bot.getAnalogGyroSensor().getAngle(AngleUnit.DEGREES));
            telemetry.addData("Realsense confidence", confidence.name());
            telemetry.addData("Target VELO (rev/s): ", bot.getTargetVelocity()/360); //convert to rev/Sec
            telemetry.addData("CURRENT THROWER 0 VELO (rev/s):", bot.getThrowerVelocity(AngleUnit.DEGREES)[0]/360);
            telemetry.addData("CURRENT THROWER 1 VELO (rev/s): ", bot.getThrowerVelocity(AngleUnit.DEGREES)[1]/360);
            telemetry.addData("Is launched: ", bot.isReadyToThrow());
            telemetry.addData("Intake AMPS:", bot.intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Intake AMPS ALEART", bot.intake.getCurrentAlert(CurrentUnit.AMPS));
            telemetry.update();
            bot.update(); //updates robot's position
            //bot.updateLightsBasedOnThrower();
            //if (bot.isInsideField(pos)) RobotLog.setGlobalWarningMessage("ROBOT POSE OFF, GO TO VUFORIA TARGET");

        }
        bot.stopVuforia();
        bot.stopRobot();
    }

    /**
     * Does the robot's tele-op.
     * Features: Field Relative Driving, Auto High Goal and Power Shots,
     * and movement of any attachments on the robot.
     */
    public void doTeleOp() {
        Pose2d pos = bot.getPoseEstimate();

        bot.updateGamepads();
            /*
            Speed Modifiers
             */
        if (bot.mode == DriveTrain6547Realsense.Mode.IDLE) {
            if (bot.x1.onPress()) speedModifier = .60;
            if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifier = .80;
            if (bot.a1.onPress() && !bot.start1.isPressed())
                speedModifier = 1.3; //sines and cosines caps speed at .7, so multiplying 1.3 balances it out by turning a .7 into a 1

            if (bot.y1.onPress()) fieldRelative.toggle(); //toggle field relative

            robotAngle = bot.getRawExternalHeading() - angleZeroValue - Math.toRadians(ANGLE_MODIFER); //angle of robot

            if (fieldRelative.output()) //if field relative is enabled
            {
                double speed = Math.hypot(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y); //get speed
                double LeftStickAngle = Math.atan2(-opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - Math.PI / 4; //get angle
                double rightX = opMode.gamepad1.right_stick_x * 2; //rotation.  Multipled by 2 to make rotation faster and smoother
                rightX *= .5; //half rotation value for better turning
                //offset the angle by the angle of the robot to make it field relative
                leftFrontPower = speed * Math.cos(LeftStickAngle - robotAngle) + rightX;
                rightFrontPower = speed * Math.sin(LeftStickAngle - robotAngle) - rightX;
                leftBackPower = speed * Math.sin(LeftStickAngle - robotAngle) + rightX;
                rightBackPower = speed * Math.cos(LeftStickAngle - robotAngle) - rightX;

                opMode.telemetry.addData("LS angle", Math.toDegrees(LeftStickAngle));
                opMode.telemetry.addData("driving toward", LeftStickAngle - robotAngle);
                opMode.telemetry.addData("ROBOT ANGLE", Math.toDegrees(robotAngle));
                opMode.telemetry.addData("RAW ANGLE", Math.toDegrees(bot.getRawExternalHeading()));
            } else //regular drive (different math because this is faster than sins and cosines
            {
                leftFrontPower = -opMode.gamepad1.left_stick_y + opMode.gamepad1.left_stick_x + opMode.gamepad1.right_stick_x;
                rightFrontPower = -opMode.gamepad1.left_stick_y - opMode.gamepad1.left_stick_x - opMode.gamepad1.right_stick_x;
                leftBackPower = -opMode.gamepad1.left_stick_y - opMode.gamepad1.left_stick_x + opMode.gamepad1.right_stick_x;
                rightBackPower = -opMode.gamepad1.left_stick_y + opMode.gamepad1.left_stick_x - opMode.gamepad1.right_stick_x;
            }
            leftFrontPower *= speedModifier;
            leftBackPower *= speedModifier;
            rightBackPower *= speedModifier;
            rightFrontPower *= speedModifier;

            opMode.telemetry.addData("leftFront Power", leftFrontPower);

            bot.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

            if (bot.rightBumper1.onPress() && bot.leftBumper1.onPress()) //calibrate gyro
            {
                //double zeroVal = -Math.toDegrees(bot.getRawExternalHeading());
                angleZeroValue = bot.getRawExternalHeading();
                opMode.telemetry.log().add("Calibrated, set zero value to " + angleZeroValue);
            }
            //interrupt all robot actions to turn toward goal.
//            if (gamepad1.left_stick_button) {
//               // bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), bot.getPoseEstimate()));
//                bot.turnRelativeSync(Math.toRadians(0));
//            }
            if (opMode.gamepad1.left_stick_button) {
                // bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), bot.getPoseEstimate()));
                bot.turnRelativeSync(Math.toRadians(0));
            }

        }

        if (opMode.gamepad1.right_stick_button && bot.mode == DriveTrain6547Realsense.Mode.IDLE) {
            bot.followTrajectory(bot.trajectoryBuilder(false, DriveSpeeds.fast).lineToLinearHeading(new Pose2d(0,-39, Math.toRadians(0))).build());
        }

        if (bot.leftTrigger1.onPress()) {
            //doPowerShots(pos);
            // bot.doRedPowerShots(pos);
            //bot.doPowerShotsTheClassicAndBetterWay();
            try {
//                Vector2d launchPos = RedLeftAutonTest.getLaunchPos();
//                bot.followTrajectorySync(bot.trajectoryBuilder()
//                .lineToLinearHeading(new Pose2d(launchPos.getX(), launchPos.getY(), Math.toRadians(0)))
//                        .build());
                doPowerShots(pos);
            } catch (Exception e) {
                RobotLog.v("PowerShot crash");
                //RobotLog.setGlobalWarningMessage("PowerShot crashed");
            }
            bot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
        if (bot.dpadDown1.onPress()) {
            doRegularShots(pos);
            bot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

        if (isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y) || isStickMoved(opMode.gamepad1.right_stick_x, opMode.gamepad1.right_stick_y)) {
            bot.mode = DriveTrain6547Realsense.Mode.IDLE;
        }

        if (opMode.gamepad2.a) {
            bot.launchRing();
            RobotLog.v("Thrower motor 0 VELO when launched: " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        } else bot.openIndexer();

        if (bot.b2.onPress()) grab.toggle();
        if (bot.y2.onPress()) lowerWobvator.toggle();

        if (grab.output()) {
            bot.grabWobbleGoal();
        } else bot.openWobbleGrabberHalfway();

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

        if (bot.leftBumper2.onPress()) {
            TurnOnThrower.toggle();
        }
        if (bot.dpadDown2.isPressed()) {
            bot.setThrowerVelocity(20 * -360, AngleUnit.DEGREES);
        }
        else if (bot.dpadDown2.onRelease() && !TurnOnThrower.output()) {
            bot.stopThrower();
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
    }

    /**
     * @param x gamepad stick X
     * @param y gamepad stick Y
     * @return weather the gamepad stick is moving
     */
    public boolean isStickMoved(double x, double y) {
        return Math.abs(x) > .3 || Math.abs(y) > .3;
    }

    /**
     * Does the High Goal shots.  Can be interrupted by moving gamepad stick
     * @param currentPos current Robot Position
     */
    public void doRegularShots(Pose2d currentPos) {
        bot.stopIntake();
        double angleToTurnTo = bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), currentPos);
        bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(new Pose2d(currentPos.getX(), currentPos.getY(), angleToTurnTo), AngleUnit.DEGREES), AngleUnit.DEGREES);

        bot.turnRelativeSync(angleToTurnTo);
        bot.turnRelativeSync(angleToTurnTo);
        //bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(currentPos, AngleUnit.DEGREES) * VELO_MUTIPLIER, AngleUnit.DEGREES);

        RobotLog.v("STARTING TO DO REGLLUAR SHOTS");
        for (int i = 0; i < 3 && !isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y); i++) {
            while (!bot.isReadyToThrow() && !isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y)) {bot.updateLightsBasedOnThrower();}
            bot.launchRing();
            RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
            if (!isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y)) sleep(500);
            bot.openIndexer();
            if (!isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y)) sleep(500);
        }
    }

    /**
     * Does 3 power shots, can be interrupted by moving gamepad stick
     * @param currentPos The current Robot position
     */
    public void doPowerShots(Pose2d currentPos) {
        bot.stopIntake();
        // Vector2d launchPos = new Vector2d(0,-14);
        //setThrowerToTarget(new Vector2d(currentPos.getX(), currentPos.getY()), Math.toRadians(0));

        //drive to middle, and face first power shot goal, prepare to launch.
//        bot.followTrajectory(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(launchPos.getX(), launchPos.getY(), Math.toRadians(0))).build());
//
//        while (!isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y) && opModeIsActive()) { bot.update();}
        double angleToTurnTo = bot.turnTowardsAngle(new Vector2d(RED_POWERSHOT_X3, FieldConstants.RED_POWER_SHOT_3Y), bot.getPoseEstimate());
        Pose2d pos = bot.getPoseEstimate();
        setThrowerToTarget(new Pose2d(pos.getX(), pos.getY(), angleToTurnTo));

        //telemetry.log().add("Drive to PowerShot");

        //turn toward 3 power shots
        RobotLog.v("Launching Power Shot 1");
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(RED_POWERSHOT_X3, FieldConstants.RED_POWER_SHOT_3Y), bot.getPoseEstimate()));
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(RED_POWERSHOT_X3, FieldConstants.RED_POWER_SHOT_3Y), bot.getPoseEstimate()));
        //throw ring
        setThrowerToTarget(bot.getPoseEstimate());
        bot.setThrowerVelocity(bot.getTargetVelocity() * PowerShot3Modifer, AngleUnit.DEGREES);
        //wait for launch speed to be ready
        while (!bot.isReadyToThrow() && !isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y)) {bot.updateLightsBasedOnThrower();}
        bot.launchRing();
        RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        if (!isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y)) sleep(500);
        bot.openIndexer();
        //prepare to throw next ring
        RobotLog.v("Launching Power Shot 2");
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(RED_POWERSHOT_X2, RED_POWERSHOT_Y2), bot.getPoseEstimate()));
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(RED_POWERSHOT_X2, RED_POWERSHOT_Y2), bot.getPoseEstimate()));
        //throw ring
        setThrowerToTarget(bot.getPoseEstimate());
        bot.setThrowerVelocity(bot.getTargetVelocity() * PowerShot2Modifer, AngleUnit.DEGREES);
        //wait for launch speed to be ready
        while (!bot.isReadyToThrow() && !isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y)) {bot.updateLightsBasedOnThrower();}
        bot.launchRing();
        RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        if (!isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y)) sleep(500);
        bot.openIndexer();
        //prepare to throw next ring
        RobotLog.v("Launching Power Shot 3");
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(RED_POWERSHOT_X1, RED_POWERSHOT_Y1), bot.getPoseEstimate()));
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(RED_POWERSHOT_X1, RED_POWERSHOT_Y1), bot.getPoseEstimate()));
        //throw ring
        setThrowerToTarget(bot.getPoseEstimate());
        bot.setThrowerVelocity(bot.getTargetVelocity() * PowerShot1Modifer, AngleUnit.DEGREES);
        //wait for launch speed to be ready
        while (!bot.isReadyToThrow() && !isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y)) {bot.updateLightsBasedOnThrower();}
        bot.launchRing();
        RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        if (!isStickMoved(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y)) sleep(500);
        bot.openIndexer();
        //stop thrower
        bot.setThrowerVelocity(0);

//        telemetry.log().add("Launched Rings");
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
            bot.setThrowerVelocity(targetRevPerSec * 360 * ThrowerUtil.POWER_SHOT_CONSTANT_TELE_OP, AngleUnit.DEGREES);
            RobotLog.v("Set power shot speed to " + (targetRevPerSec*360) + "Rev/s, dist (in): " + dist + ", distX: " + (startPos.getX() - FieldConstants.TOP_OF_FIELD) + ", distY: " + (startPos.getY() - targetY) + " targetY: " + targetY);
        }
        else {
            double vi = ThrowerUtil.getVi(startPos.getX(), ThrowerUtil.INITIAL_HEIGHT, dist, FieldConstants.RED_GOAL_HEIGHT, ThrowerUtil.INITIAL_ANGLE);
            double targetRevPerSec = vi / ThrowerUtil.inchesPerRev;
            bot.setThrowerVelocity(targetRevPerSec * 360, AngleUnit.DEGREES);
            RobotLog.v("Set speed to " + (targetRevPerSec*360) + "Rev/s");
        }
    }


}
