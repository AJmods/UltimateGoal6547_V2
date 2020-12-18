package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.drivetrain.DriveSpeeds;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.pipeline.openCvPipeLines;
import org.firstinspires.ftc.teamcode.util.throwerUtil;

@Config
@Autonomous(name = "Red Left Auton", group = "test")
public class RedLeftAutonTest extends LinearOpMode {

    DriveTrain6547Realsense bot;
    openCvPipeLines.RingCount ringCount = openCvPipeLines.RingCount.NONE;

    public static int enumRing = 0;

    public static boolean isBlue = false;

    private Pose2d startPos = new Pose2d(-56,-25);

    enum BotStartColor{
        RED, BLUE
    }
    enum BotStartDirection {
        LEFT, RIGHT
    }

//    public static BotStartColor botStartColor;
//    public static BotStartDirection botStartDirection;

    @Override
    public void runOpMode() throws InterruptedException {

        bot = new DriveTrain6547Realsense(this);
        bot.setPoseEstimate(startPos);

        bot.initOpenCV();
        bot.startOpenCV();

        bot.grabWobbleGoal();

        ringCount = openCvPipeLines.RingCount.values()[enumRing];

        bot.update();
        telemetry.log().add("Ready to Start");
        while (!isStarted() && !isStopRequested()) {
            ringCount = bot.getRingCount();
            telemetry.addData("Ring Count", ringCount);
            telemetry.update();
        }
        waitForStart();
        bot.stopOpenCV();

        //bot.lowerWobvatorByNotAllTheWay();
        bot.openIndexer();

        Vector2d launchPos = new Vector2d(0,-14);
        setThrowerToTarget(launchPos, Math.toRadians(0));

        //drive to middle, and face first power shot goal, prepare to launch.
        bot.followTrajectorySync(bot.trajectoryBuilder().lineTo(launchPos).build());
        bot.lowerWobvatorByNotAllTheWay();
        setThrowerToTarget(bot.getPoseEstimate());
        sleep(500);

        telemetry.log().add("Drive to PowerShot");

        //turn toward 3 power shots
        RobotLog.v("Launching Power Shot 1");
        bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y), bot.getPoseEstimate()));
        bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y), bot.getPoseEstimate()));
        //throw ring
        setThrowerToTarget(bot.getPoseEstimate());
        sleep(500);
        bot.launchRing();
        RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        sleep(500);
        bot.openIndexer();
        //prepare to throw next ring
        RobotLog.v("Launching Power Shot 2");
        bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y), bot.getPoseEstimate()));
        bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y), bot.getPoseEstimate()));
        //throw ring
        setThrowerToTarget(bot.getPoseEstimate());
        sleep(500);
        bot.launchRing();
        RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        sleep(500);
        bot.openIndexer();
        //prepare to throw next ring
        RobotLog.v("Launching Power Shot 3");
        bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), bot.getPoseEstimate()));
        bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), bot.getPoseEstimate()));
        //throw ring
        setThrowerToTarget(bot.getPoseEstimate());
        sleep(500);
        bot.launchRing();
        RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
        sleep(500);
        bot.openIndexer();
        //stop thrower
        bot.setThrowerVelocity(0);

        telemetry.log().add("Launched Rings");

        bot.lowerWobvatorByNotAllTheWay();

        if (ringCount == openCvPipeLines.RingCount.NONE) bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(28, -42, Math.toRadians(0))).build());
        else if (ringCount == openCvPipeLines.RingCount.ONE) bot.followTrajectorySync(bot.trajectoryBuilder().splineTo(new Vector2d(45,-17), Math.toRadians(0)).build());
        else if (ringCount == openCvPipeLines.RingCount.FOUR) bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.fast).splineTo(new Vector2d(65,-42), Math.toRadians(0)).build());

        telemetry.log().add("Drove to target goal");
        bot.releaseWobbleGoal();

        //grab other wobble goal
        sleep(500);
        bot.raiseWobvator();

        //go to wobble goal
        if (ringCount == openCvPipeLines.RingCount.NONE)  {bot.followTrajectorySync(bot.trajectoryBuilder(true, DriveSpeeds.fast).splineTo(new Vector2d(-40, -30), Math.toRadians(180))
                .build());

        }else{ bot.followTrajectorySync(bot.trajectoryBuilder(true, DriveSpeeds.fast).splineTo(new Vector2d(-30, -17), Math.toRadians(180)).build());

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .lineToLinearHeading(new Pose2d(-40, -32, Math.toRadians(0)))
                .build());
        }

        bot.lowerWobvator();
        bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.slow).lineToConstantHeading(new Vector2d(-40,-34)).build());

        //grab wobble goal
        sleep(500);
        bot.grabWobbleGoal();
        sleep(750);
        bot.lowerWobvatorByNotAllTheWay();

        telemetry.log().add("Going to Drive to Area");

        //then drive  back to orginal sqaure
        if (ringCount == openCvPipeLines.RingCount.NONE) {
            bot.followTrajectorySync(bot.trajectoryBuilder().splineTo(new Vector2d(21, -42), Math.toRadians(0)).build());
        }
        else if (ringCount == openCvPipeLines.RingCount.ONE) {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .lineToLinearHeading(new Pose2d(-30,-14, Math.toRadians(0)))
                    .build());

            bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.fast)
                   .splineTo(new Vector2d(40, -14), Math.toRadians(0))
                    .build());
        } else if (ringCount == openCvPipeLines.RingCount.FOUR) {
            bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.fast)
                    .lineToLinearHeading(new Pose2d(-30,-14, Math.toRadians(0)))
                    .build());

            bot.followTrajectorySync(bot.trajectoryBuilder(false, DriveSpeeds.fast)
                    .splineTo(new Vector2d(63, -42), Math.toRadians(0))
                    .build());
        }

        //drop wobble goal
        sleep(250);
        bot.releaseWobbleGoal();
        sleep(750);
        bot.raiseWobvator();

        if (ringCount == openCvPipeLines.RingCount.ONE) {bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(13,-12 ,Math.toRadians(0))).build());}
        else bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(new Pose2d(13,-36, Math.toRadians(0))).build());

        bot.turnRealtiveSync(Math.toRadians(0));
        bot.turnRealtiveSync(Math.toRadians(0));
        sleep(250);

        bot.savePos(bot.getPoseEstimate());
        telemetry.log().add("AUTON IS DONE");
        RobotLog.v("AUTON IS DONE");
        bot.setPacketAction(((packet, fieldOverlay) -> packet.addLine("AUTON IS DONE")));
        bot.update();

        //sleep(2000);

        //go back to beginning
      //  bot.followTrajectorySync(bot.trajectoryBuilder().lineToLinearHeading(startPos).build());
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
        double targetY = throwerUtil.getTargetY(startPos, FieldConstants.TOP_OF_FIELD);
        double dist = Math.hypot(startPos.getX() - FieldConstants.TOP_OF_FIELD, startPos.getY() - targetY);
        if (isPowerShot) {
            double vi = throwerUtil.getVi(startPos.getX(), throwerUtil.INITAL_HEIGHT, dist, FieldConstants.POWER_SHOT_HEIGHT, throwerUtil.INITAL_ANGLE);
            double targetRevPerSec = vi / throwerUtil.inchesPerRev;
            bot.setThrowerVelocity(targetRevPerSec * 360 * throwerUtil.POWER_SHOT_CONSTANT, AngleUnit.DEGREES);
            RobotLog.v("Set speed to " + (targetRevPerSec*360) + "Rev/s");
        }
        else {
            double vi = throwerUtil.getVi(startPos.getX(), throwerUtil.INITAL_HEIGHT, dist, FieldConstants.RED_GOAL_HEIGHT, throwerUtil.INITAL_ANGLE);
            double targetRevPerSec = vi / throwerUtil.inchesPerRev;
            bot.setThrowerVelocity(targetRevPerSec * 360, AngleUnit.DEGREES);
            RobotLog.v("Set speed to " + (targetRevPerSec*360) + "Rev/s");
        }
    }
}
