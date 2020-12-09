package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.drivetrain.localizer.T265LocalizerRR;
import org.firstinspires.ftc.teamcode.util.MiniPID;

@TeleOp
@Config
@Disabled
public class MiniPIDTurnTest extends LinearOpMode {

    MiniPID headingPID = new MiniPID(0,0,0);
    MiniPID xPID = new MiniPID(0,0,0);
    MiniPID yPID = new MiniPID(0,0,0);
    public static double tP = 0;
    public static double tI = 0;
    public static double tD = 0;
//    public static double xP = 0;
//    public static double xI = 0;
//    public static double xD = 0;
//    public static double yP = 0;
//    public static double yI = 0;
//    public static double yD = 0;
    public static double ANGLE = 0;
//    public static double XTARGET = 0;
//    public static double YTARGET = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        headingPID.setOutputLimits(0,1);
        xPID.setOutputLimits(0,1);
        yPID.setOutputLimits(0,1);
        telemetry.log().add("ready to start");
        waitForStart();

        while (opModeIsActive()) {
            headingPID.setD(tD);
            headingPID.setI(tI);
            headingPID.setP(tP);
//            xPID.setP(xP);
//            xPID.setI(xI);
//            xPID.setD(xD);
//            yPID.setP(yP);
//            yPID.setI(yI);
//            yPID.setD(yD);

            double power = headingPID.getOutput(T265LocalizerRR.getHeading(), Math.toRadians(ANGLE));

            bot.setMotorPowers(power,power,-power,-power);

            bot.setPacketAction((packet, fieldOverlay) -> {
                packet.put("POWER", power);
                packet.put("ANGLE", Math.toRadians(ANGLE));
                packet.put("ACTUAL ANGLE", T265LocalizerRR.getHeading());
            });

            bot.update();



        }


    }
}
