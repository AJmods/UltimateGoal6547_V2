package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.util.MiniPID;

@TeleOp
@Config
public class MiniPIDTesting extends LinearOpMode {

    DriveTrain6547Realsense bot;

    public static double targetX=0;
    public static double targetY=0;
    public static double targetAngleDeg=0;

    public static double kP_x = 0;
    public static double kI_x = 0;
    public static double kD_x = 0;

    public static double kP_y = 0;
    public static double kI_y = 0;
    public static double kD_y = 0;

    public static double kP_theta = 0;
    public static double kI_theta = 0;
    public static double kD_theta = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new DriveTrain6547Realsense(this);

        MiniPID xPID = new MiniPID(kP_x, kI_x, kD_x);
        MiniPID yPID = new MiniPID(kP_x, kI_x, kD_x);
        MiniPID thetaPID = new MiniPID(kP_x, kI_x, kD_x);

        xPID.setOutputLimits(-1,1);
        yPID.setOutputLimits(-1,1);
        thetaPID.setOutputLimits(-1,1);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.log().add("Ready to start");
        waitForStart();

        while (opModeIsActive()) {
            Pose2d pos = bot.getPoseEstimate();

            xPID.setP(kP_x);
            xPID.setI(kI_x);
            xPID.setD(kD_x);

            yPID.setP(kP_y);
            yPID.setI(kI_y);
            yPID.setD(kD_y);

            thetaPID.setP(kP_theta);
            thetaPID.setI(kI_theta);
            thetaPID.setD(kD_theta);

            double xOutput = xPID.getOutput(pos.getX(), targetX);
            double yOutput = yPID.getOutput(pos.getY(), targetY);
            double thetaOutput = thetaPID.getOutput(pos.getHeading(), Math.toRadians(targetAngleDeg));

            double speed = Math.hypot(xOutput, yOutput); //get speed
            double LeftStickAngle = Math.atan2(xOutput, yOutput) - Math.PI / 4; //get angle
            double rightX = thetaOutput; //rotation.  Multipled by 2 to make rotation faster and smoother

            //offset the angle by the angle of the robot to make it field relative
            double leftFrontPower = speed * Math.cos(LeftStickAngle - pos.getHeading()) + rightX;
            double rightFrontPower = speed * Math.sin(LeftStickAngle - pos.getHeading()) - rightX;
            double leftBackPower = speed * Math.sin(LeftStickAngle - pos.getHeading()) + rightX;
            double rightBackPower = speed * Math.cos(LeftStickAngle - pos.getHeading()) - rightX;

            bot.setMotorPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

            telemetry.addData("target X", targetX);
            telemetry.addData("Actual X", pos.getX());
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Actual Y", pos.getY());
            telemetry.addData("Target Angle (deg)", targetAngleDeg);
            telemetry.addData("Actual Angle (deg)", Math.toDegrees(pos.getHeading()));
            telemetry.update();

        }
    }
}
