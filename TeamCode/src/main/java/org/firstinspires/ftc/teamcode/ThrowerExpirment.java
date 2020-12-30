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
import org.firstinspires.ftc.teamcode.util.ThrowerUtil;

/*
This is the tele-op we use to drive the robot
 */
@Config
@TeleOp(name = "Thrower Experiment", group = "_teleOp")
public class ThrowerExpirment extends LinearOpMode {

    public static double REV_PER_SEC = 44;
    public static double TARGET_HIGHET = FieldConstants.RED_GOAL_HEIGHT;

    public static boolean LOCK_BOT_TO_POS = true;
    public static double ROBOT_X = 0;
    public static double ROBOT_Y = 0;
    public static double ROBOT_HEADING_DEG = 0;

    public static boolean TURN_ON_THROWER = false;

    public static double launchRing = 0;

    private final double RED_GOAL_X = FieldConstants.RED_GOAL_X;

    private ToggleBoolean feildRealtive = new ToggleBoolean(true);
    ToggleBoolean grab = new ToggleBoolean(false);
    ToggleBoolean lowerWobwater = new ToggleBoolean(false);
    ToggleInt powerShot = new ToggleInt(3);

    private DriveTrain6547Realsense bot; //the robot class

    private double angleZeroValue = 0;

    private double robotAngle=0;

    private double veloWhenLaunchedRevPerSec = 0;

    @Override
    public void runOpMode() throws InterruptedException{
       // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //makes telemetry output to the FTC Dashboard
        bot = new DriveTrain6547Realsense(this, true);
        telemetry.update();

        telemetry.log().add("Ready to start");
        waitForStart();

        while (opModeIsActive()) {

            Pose2d pos = bot.getPoseEstimate();

            bot.updateGamepads();
            /*
            Speed Modifers
             */
            if (gamepad1.left_stick_button) {
               // bot.turnRealtiveSync(bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), bot.getPoseEstimate()));
                bot.turnRealtiveSync(Math.toRadians(0));
            }

            if (gamepad2.a) { bot.launchRing();
                veloWhenLaunchedRevPerSec =  (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360);
                RobotLog.v("Thrower motor 0 VELO when lauched: " + (bot.getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
            } else bot.openIndexer();

            boolean isValidAngle = ThrowerUtil.isValidAngle(pos.getX(), pos.getY(), pos.getHeading());

            double targetY = ThrowerUtil.getTargetY(pos, RED_GOAL_X);
            double deltaX = pos.getX() - RED_GOAL_X;
            double detlaY = pos.getY() - targetY;
            double dist = Math.hypot(deltaX, detlaY);
            double vi = ThrowerUtil.getVi(0, ThrowerUtil.INITAL_HEIGHT, dist, TARGET_HIGHET, ThrowerUtil.INITAL_ANGLE);
            double targetRev = vi/ ThrowerUtil.inchesPerRev;


            if (TURN_ON_THROWER) {
                    bot.setThrowerVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);
                } else {
                   bot.setThrowerVelocity(0);
                }

                if (bot.mode == DriveTrain6547Realsense.Mode.IDLE && LOCK_BOT_TO_POS) {
                    try {
                        bot.followTrajectory(bot.trajectoryBuilder(false, DriveSpeeds.slow).lineToLinearHeading(new Pose2d(ROBOT_X, ROBOT_Y, Math.toRadians(ROBOT_HEADING_DEG))).build());
                    } catch (Exception e) {
                        RobotLog.v("Robot Already at Target POS");
                    }
                }

            bot.setPacketAction((packet, fieldOverlay) -> {

                packet.addLine("DISTANCE: " + dist);
                packet.addLine("TARGET REV/S: " + REV_PER_SEC);

                //add data.
                packet.addLine("Actaul Ring Velocity when Launched: " + veloWhenLaunchedRevPerSec + " REV/s");
                packet.addLine("Theoretical Ring Velocity: " + targetRev + " REV/s");

                double drawsConstant = REV_PER_SEC/targetRev;
                packet.addLine("Current Motor Rev/s)/(Target Rev/s): " + drawsConstant + " (Danda's Constant)");

                packet.addLine("INITIAL ANGLE: " + ThrowerUtil.INITAL_ANGLE);
                packet.addLine("INITIAL HEIGHT: " + ThrowerUtil.INITAL_HEIGHT);
                //draw robot
              //  DashboardUtil.drawRobot(fieldOverlay, pos);
                //draw launcherPos
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