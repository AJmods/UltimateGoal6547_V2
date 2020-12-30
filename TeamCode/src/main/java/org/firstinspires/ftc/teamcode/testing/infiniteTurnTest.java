package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.drivetrain.localizer.T265LocalizerRR;

@Config
@TeleOp
@Disabled
public class infiniteTurnTest extends LinearOpMode {

    public static double TARGET_ANGLE = 0;
    public static double targetX=0;
    public static double targetY=0;
    private DriveTrain6547Realsense bot;

    @Override
    public void runOpMode() throws InterruptedException {
       bot = new DriveTrain6547Realsense(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        double lastAngle = -1;

        while (opModeIsActive()) {
            Vector2d targetPos = new Vector2d(targetX, targetY);
            Pose2d robotPos = bot.getPoseEstimate();
            double targetAngleRad = bot.turnTowardsAngle(targetPos, robotPos); //Math.toRadians(TARGET_ANGLE);
                targetAngleRad -= T265LocalizerRR.getHeading();
                if (Math.abs(targetAngleRad) > Math.toRadians(180)) //make the angle difference less then 180 to remove unnecessary turning
                {
                    targetAngleRad += (targetAngleRad >= 0) ? Math.toRadians(-360) : Math.toRadians(360);
                }
                bot.turn(targetAngleRad, bot.getCurrentHeadingV(), bot.getCurrentHeadingA(), 0);

            double finalTargetAngleRad = targetAngleRad;
            double finalLastAngle = lastAngle;
            bot.setPacketAction((packet, fieldOverlay) -> {
                    packet.put("Target POS: ", targetPos.toString());
                    packet.put("Target Angle: ", Math.toDegrees(finalTargetAngleRad));
                    packet.put("Last Angle", finalLastAngle);
                    fieldOverlay.fillCircle(targetX,targetY,4);
                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.strokeLine(robotPos.getX(), robotPos.getY(), targetX, targetY);
                });
           // }
//            else if (bot.mode == DriveTrain6547Realsense.Mode.IDLE && InfiniteTurn && T265Localizer.getHeading() < targetAngleRad - LEEWAY && T265Localizer.getHeading() > targetAngleRad + LEEWAY) {
//                targetAngleRad-=T265Localizer.getHeading();
//                if (Math.abs(targetAngleRad)>Math.toRadians(180)) //make the angle difference less then 180 to remove unnecessary turning
//                {
//                    targetAngleRad+=(targetAngleRad>=0) ? Math.toRadians(-360) : Math.toRadians(360);
//                }
//                bot.turn(targetAngleRad);
//            }
            bot.update();
        }


    }

    double norm(double angle) {
        while (angle >= Math.toRadians(180)) angle-=Math.toRadians(180);
        while (angle <Math.toRadians(-180)) angle+=Math.toRadians(180);
        return angle;
    }

}
