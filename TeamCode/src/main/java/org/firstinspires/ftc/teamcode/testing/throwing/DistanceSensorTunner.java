package org.firstinspires.ftc.teamcode.testing.throwing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@TeleOp
@Config
@Disabled
public class DistanceSensorTunner extends LinearOpMode {

    public static double MULITPLIER = 1;
    public static double DIST_ADD = -9;
    @Override
    public void runOpMode() throws InterruptedException {

        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        while (opModeIsActive()) {
            double trueDistance = bot.distanceSensorX.getVoltage();
            double realDistance = getRealDistance(bot.getPoseEstimate()) + DIST_ADD;

            MULITPLIER = realDistance/trueDistance;

            bot.update();

            telemetry.addData("MULIPLIER", MULITPLIER);
            telemetry.addData("distance Sensor Voltage", trueDistance);
            telemetry.addData("Dstance Sensor Distance", getDistance(bot.distanceSensorX));
            telemetry.addData("real Distance", realDistance - DIST_ADD);
            telemetry.addData("Max Voltage", bot.distanceSensorX.getMaxVoltage());
            telemetry.update();
        }

    }

    public double getDistance(AnalogInput distanceSensor) {
        return distanceSensor.getVoltage()*MULITPLIER;
    }
    public double getRealDistance(Pose2d pos) {
        return 24.5-pos.getY();
    }
}
