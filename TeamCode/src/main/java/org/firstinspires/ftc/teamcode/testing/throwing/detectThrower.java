package org.firstinspires.ftc.teamcode.testing.throwing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@TeleOp
@Config
public class detectThrower extends LinearOpMode {

    public static double REV_PER_SEC = 0;
    public static double LEEWAY = .5;


    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        telemetry.log().add("Ready to Start");
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            bot.setThrowerVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);
            if (gamepad1.a) {
                bot.launchRing();
            } else bot.midIndexer();

            double leewayDeg = LEEWAY * 360;

            double[] revPerSec = bot.getThrowerVelocity();
            for (int i = 0; i < revPerSec.length; i++) revPerSec[i]/=360;

            boolean isMotor1TargetVelo;
            boolean isMotor2TargetVelo;

            isMotor1TargetVelo = revPerSec[0] > REV_PER_SEC-leewayDeg && revPerSec[0] < REV_PER_SEC + leewayDeg;
            isMotor2TargetVelo = revPerSec[1] > REV_PER_SEC-leewayDeg && revPerSec[1] < REV_PER_SEC + leewayDeg;

            boolean launched = isMotor1TargetVelo || isMotor2TargetVelo;

            telemetry.addData("Motor 1 REV/s", "%.2f", revPerSec[0]);
            telemetry.addData("Motor 2 REV/2", "%.2f", revPerSec[1]);
            telemetry.addData("Motor AMPS", bot.thrower1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor 2 AMPS", bot.thrower2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("IS MOTOR 1 AT TARGET: ", isMotor1TargetVelo);
            telemetry.addData("IS MOTOR 2 AT TARGET: ", isMotor2TargetVelo);
            telemetry.addData("isLaunched: ", isMotor1TargetVelo);
        }

    }
}
