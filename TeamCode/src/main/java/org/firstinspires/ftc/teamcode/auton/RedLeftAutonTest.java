package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Bot2;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.drivetrain.DriveSpeeds;
import org.firstinspires.ftc.teamcode.drivetrain.localizer.T265LocalizerRR;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.command.PacketAction;
import org.firstinspires.ftc.teamcode.util.pipeline.openCvPipeLines;
import org.firstinspires.ftc.teamcode.util.ThrowerUtil;
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil;

/**
 * Red Left Autonomous
 * @see DriveTrain6547Realsense
 */
@Config
@Autonomous(name = "Red Left Auton", group = "test")
public class RedLeftAutonTest extends LinearOpMode {

    Bot2 bot; //the robot

    //this defaults to no rings detected
    openCvPipeLines.RingCount ringCount = openCvPipeLines.RingCount.NONE;
    //robot starting position
    private static Pose2d startPos = new Pose2d(-52,-25);
    //location where robot launches powershots
    private static Vector2d launchPos = new Vector2d(0,-14);

    public double encoderTicksPerConveyor = 500;

    @Override
    public void runOpMode() throws InterruptedException {

        bot = new Bot2(this);
        T265LocalizerRR.slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(startPos.getX(), startPos.getY()), new Rotation2d(startPos.getHeading())));
        bot.setPoseEstimate(startPos);

        bot.thrower2.setDirection(DcMotorSimple.Direction.FORWARD);
        bot.thrower1.setDirection(DcMotorSimple.Direction.FORWARD);

        bot.reverseMotors();

        //init bot
        bot.raiseWobvator();
        bot.openIndexer();
        bot.grabWobbleGoal();

        bot.initOpenCV();
        bot.startOpenCV();

        bot.grabWobbleGoal();

        bot.update();
        telemetry.log().add("Ready to Start");


        boolean messageDisplayed = false;
        while (!isStarted() && !isStopRequested()) {
            ringCount = bot.getRingCount();
            telemetry.addData("Ring Count", ringCount);
            telemetry.addData("distance", getRobotX());
            telemetry.addData("Robot Velocity", bot.getPoseVelocity().toString());

            Pose2d distPos = new Pose2d(-56, getRobotX(), Math.toRadians(0));
//
//            if (distPos.getY() > startPos.getY() + 1 || distPos.getY() < startPos.getY() - 1) {
//                RobotLog.setGlobalWarningMessage("ROBOT IS POSITIONED WRONG at pos %s", distPos.toString());
//                messageDisplayed = true;
//            } else if (messageDisplayed) {
//                RobotLog.clearGlobalWarningMsg();
//                messageDisplayed = false;
//            }

            bot.setPacketAction((packet, fieldOverlay) -> {
                packet.addLine("Ring Count: " + ringCount);
                fieldOverlay.setStroke("#42F578"); //lime green+
                DashboardUtil.drawRobot(fieldOverlay, distPos);

                packet.addLine("Distance");
            });
            if (!bot.isInsideField(bot.getPoseEstimate())) {
                bot.setPoseEstimate(startPos);
                RobotLog.clearGlobalWarningMsg();
            }
            bot.update();
            telemetry.update();
        }
        waitForStart();
        bot.stopOpenCV();

        //bot.lowerWobvatorByNotAllTheWay();
        bot.openIndexer();

        setThrowerToTarget(launchPos, Math.toRadians(0));

        //drive to middle, and face first power shot goal, prepare to launch.
        bot.followTrajectorySync(bot.trajectoryBuilder().lineTo(launchPos).build());
       // setThrowerToTarget(bot.getPoseEstimate());
        sleep(500);

        telemetry.log().add("Drive to PowerShot");

        //turn toward 3 power shots
        RobotLog.v("Launching Power Shot 1");
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y), bot.getPoseEstimate()));
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y), bot.getPoseEstimate()));
        //throw ring
        bot.setThrowerVelocity(1400);
        //setThrowerToTarget(bot.getPoseEstimate());
        //wait for launch speed to be ready
        //while (!bot.isReadyToThrow() && opModeIsActive()) {bot.updateLightsBasedOnThrower();}
        bot.zeroConveyor();
        bot.launchRing();
        while (bot.conveyor.getCurrentPosition() < encoderTicksPerConveyor && opModeIsActive()) {

        }
        telemetry.log().add("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + " REV/s");
        bot.stopLaunch();
        //prepare to throw next ring
        RobotLog.v("Launching Power Shot 2");
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y), bot.getPoseEstimate()));
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y), bot.getPoseEstimate()));
        //throw ring
        sleep(250);
        bot.setThrowerVelocity(1300);
        //setThrowerToTarget(bot.getPoseEstimate());
        //wait for launch speed to be ready

        //while (!bot.isReadyToThrow() && opModeIsActive()) {bot.updateLightsBasedOnThrower();}
        bot.zeroConveyor();
        bot.launchRing();
        while (bot.conveyor.getCurrentPosition() < encoderTicksPerConveyor && opModeIsActive()) {

        }
        telemetry.log().add("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + " REV/s");
        bot.stopLaunch();
        //prepare to throw next ring
        RobotLog.v("Launching Power Shot 3");
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), bot.getPoseEstimate()));
        bot.turnRelativeSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), bot.getPoseEstimate()));
        //throw ring
        //setThrowerToTarget(bot.getPoseEstimate());
        sleep(250);
        bot.setThrowerVelocity(1300);
        //wait for launch speed to be ready
        //while (!bot.isReadyToThrow() && opModeIsActive()) {bot.updateLightsBasedOnThrower();}
        bot.zeroConveyor();
        bot.launchRing();
        while (bot.conveyor.getCurrentPosition() < encoderTicksPerConveyor && opModeIsActive()) {

        }
        telemetry.log().add("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + " REV/s");
        bot.stopLaunch();
        //stop thrower
        bot.setThrowerVelocity(0);

        telemetry.log().add("Launched Rings");

        bot.lowerWobvatorByNotAllTheWay();

        if (ringCount == openCvPipeLines.RingCount.NONE) bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(20, -45, Math.toRadians(0))).build());
        else if (ringCount == openCvPipeLines.RingCount.ONE) bot.followTrajectorySync(bot.trajectoryBuilder().splineTo(new Vector2d(43,-23), Math.toRadians(0)).build());
        else if (ringCount == openCvPipeLines.RingCount.FOUR) bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.reallyFast).splineTo(new Vector2d(60,-46), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    bot.releaseWobbleGoal();
                    bot.mode = DriveTrain6547Realsense.Mode.IDLE;
                })
                //.splineTo(new Vector2d(79, -49), Math.toRadians(0))
                .build());

        telemetry.log().add("Drove to target goal");
        bot.openWobbleGrabberHalfway();

        //grab other wobble goal
        sleep(100);
        //bot.raiseWobvator();

        //go to wobble goal
        bot.lowerWobvatorByNotAllTheWay();
        if (ringCount == openCvPipeLines.RingCount.NONE)  {bot.followTrajectorySync(bot.trajectoryBuilder(true, DriveSpeeds.reallyFast).splineTo(new Vector2d(-40, -28), Math.toRadians(180))
                .build());

        }else {
            bot.outtake(); //actually intakes
            bot.followTrajectorySync(bot.trajectoryBuilder(true, DriveSpeeds.reallyFast)
                //.splineTo(new Vector2d(-30, -17), Math.toRadians(180))
                .splineTo(new Vector2d(-40,-20), Math.toRadians(180)).build());
            bot.stopRobot();

//        bot.followTrajectorySync(bot.trajectoryBuilder()
//                .lineToLinearHeading(new Pose2d(-40, -32, Math.toRadians(0)))
//                .build());
       }

        bot.lowerWobvator();
        bot.followTrajectorySync(bot.trajectoryBuilder(false).lineToConstantHeading(new Vector2d(-39,-33)).build());

        //grab wobble goal
        sleep(250);
        bot.grabWobbleGoal();
        sleep(750);
        bot.lowerWobvatorByNotAllTheWay();

        telemetry.log().add("Going to Drive to Area");

        //then drive  back to original square
        if (ringCount == openCvPipeLines.RingCount.NONE) {
            bot.followTrajectorySync(bot.trajectoryBuilder().splineTo(new Vector2d(18, -40), Math.toRadians(0)).build());
        }
        else if (ringCount == openCvPipeLines.RingCount.ONE) {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .lineToLinearHeading(new Pose2d(-30,-14, Math.toRadians(0)))
                    .build());

            bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.reallyFast)
                   .splineTo(new Vector2d(38, -20), Math.toRadians(0))
                    .build());
        } else if (ringCount == openCvPipeLines.RingCount.FOUR) {
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .lineToConstantHeading(new Vector2d(-20, -20))
            .build());
            bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.reallyFast)
                    //.splineTo(new Vector2d(-30,-14), Math.toRadians(0))
                    .splineTo(new Vector2d(60, -42), Math.toRadians(0))
                    .addDisplacementMarker(() -> {
                        bot.releaseWobbleGoal();
                        bot.mode = DriveTrain6547Realsense.Mode.IDLE;
                    })
                    //.splineTo(new Vector2d(83, -45), Math.toRadians(0))
                    .build());


//            bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.reallyFast)
//                    .splineTo(new Vector2d(64, -40), Math.toRadians(0))
//                    .build());
        }

        //drop wobble goal
       // sleep(250);

        bot.releaseWobbleGoal();
        //sleep(250);
        bot.raiseWobvator();

        if (ringCount == openCvPipeLines.RingCount.ONE) {bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(13,-12 ,Math.toRadians(0))).build());}
        else if (ringCount == openCvPipeLines.RingCount.NONE || ringCount == openCvPipeLines.RingCount.FOUR) bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(13,-36, Math.toRadians(0))).build());
//        else {
//            double angleToTurnTo = bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), new Pose2d(5,-36));
//            bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(new Pose2d(5,-36, angleToTurnTo), AngleUnit.DEGREES), AngleUnit.DEGREES);
//            bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(5,-36, Math.toRadians(0))).build());
//        }

//        if (ringCount == openCvPipeLines.RingCount.FOUR) {
//            doRegularShots(bot.getPoseEstimate(), 4);
//        }
        bot.stopIntake();
        bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(10,-42,Math.toRadians(-90))).build());

        bot.turnRelativeSync(Math.toRadians(-90));
        bot.turnRelativeSync(Math.toRadians(-90));
        sleep(100);

        bot.savePos(bot.getPoseEstimate());
        telemetry.log().add("AUTON IS DONE");
        RobotLog.v("RED AUTON IS DONE");
        bot.setPacketAction(((packet, fieldOverlay) -> packet.addLine("RED AUTON IS DONE")));
        bot.update();

        //sleep(2000);

        //go back to beginning
      //  bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(startPos).build());
    }
    public void doRegularShots(Pose2d currentPos, int numberOfRings) {
        bot.stopIntake();
        double angleToTurnTo = bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), currentPos);
        bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(new Pose2d(currentPos.getX(), currentPos.getY(), angleToTurnTo), AngleUnit.DEGREES), AngleUnit.DEGREES);

        bot.turnRelativeSync(angleToTurnTo);
        bot.turnRelativeSync(angleToTurnTo);
        //bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(currentPos, AngleUnit.DEGREES) * VELO_MUTIPLIER, AngleUnit.DEGREES);

        for (int i = 0; i < numberOfRings && opModeIsActive(); i++) {
            while (!bot.isReadyToThrow() && opModeIsActive()) {bot.updateLightsBasedOnThrower();}
            bot.launchRing();
            RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
            sleep(400);
            bot.openIndexer();
            sleep(400);
        }
        bot.stopThrower();
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

    public double getRobotX() {
        double dist = 24 - bot.getDistance(bot.distanceSensorX);
        double x_dist_add = -5.5;
        return x_dist_add + dist;
    }

    public static void setLaunchPos(Vector2d launchPos) {
        RedLeftAutonTest.launchPos = launchPos;
    }

    public static void setStartPos(Pose2d startPos) {
        RedLeftAutonTest.startPos = startPos;
    }

    public static Pose2d getStartPos() {
        return startPos;
    }

    public static Vector2d getLaunchPos() {
        return launchPos;
    }
}
