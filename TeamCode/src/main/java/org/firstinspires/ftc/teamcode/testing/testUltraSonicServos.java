package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.teleOp.LeagueChampionshipTeleop;
import org.firstinspires.ftc.teamcode.util.UltraSonicServo;
import org.firstinspires.ftc.teamcode.util.command.PacketAction;
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil;

@TeleOp
@Config
public class testUltraSonicServos extends LinearOpMode {

    DriveTrain6547Realsense bot;
    LeagueChampionshipTeleop leagueChampionshipTeleop;

    UltraSonicServo ultraSonicServoX;
    UltraSonicServo ultraSonicServoY;

    public static double realSenseX = 9;
    public static double realSenseY = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = new DriveTrain6547Realsense(this);

        leagueChampionshipTeleop = new LeagueChampionshipTeleop(this, bot);

        ultraSonicServoX = new UltraSonicServo(bot.distanceSensorServoX, bot.distanceSensorX, new Pose2d(-5.5,0,Math.toRadians(0)));
        ultraSonicServoY = new UltraSonicServo(bot.distanceSensorServoY, bot.distanceSensorY, new Pose2d(5.5,0,Math.toRadians(0)));


        telemetry.log().add("Ready to Start");
        waitForStart();

        while (opModeIsActive()) {
            Pose2d pos = bot.getPoseEstimate();
            ultraSonicServoX.adjustServoBasedOnRotation(pos.getHeading());
            ultraSonicServoY.adjustServoBasedOnRotation(pos.getHeading());

            Pose2d newPose = calculatePositionBasedOnUltraSonicServos(ultraSonicServoX, ultraSonicServoY, pos.getHeading(), new Vector2d(realSenseX, realSenseY));

            telemetry.addData("Modified Angle", (pos.getHeading()>Math.toRadians(180)) ? pos.getHeading()-Math.toRadians(360) : pos.getHeading());
            telemetry.addData("new POSE: ", newPose.toString());
            telemetry.addData("servo pos: ", ultraSonicServoX.getServo().getPosition());
            telemetry.addData("servo Y pos: ", ultraSonicServoY.getServo().getPosition());
            telemetry.addData("distance X (in)", ultraSonicServoX.getRawDistance(DistanceUnit.INCH));
            telemetry.addData("distance Y (in)", ultraSonicServoY.getRawDistance(DistanceUnit.INCH));
            telemetry.addData("Y voltage", ultraSonicServoY.getDistanceSensor().getVoltage());
            telemetry.addData("Y voltage again", bot.distanceSensorY.getVoltage());
            bot.setPacketAction((packet, fieldOverlay) -> {
                fieldOverlay.setStroke("0000FF");
                DashboardUtil.drawRobot(fieldOverlay, newPose);
            });
            telemetry.update();
            leagueChampionshipTeleop.doTeleOp();
            bot.update();
        }
    }

    public Pose2d calculatePositionBasedOnUltraSonicServos(UltraSonicServo ultraSonicServoX, UltraSonicServo ultraSonicServoY, double robotHeading) {
        return calculatePositionBasedOnUltraSonicServos(ultraSonicServoX, ultraSonicServoY, robotHeading, new Vector2d());
    }
    public Pose2d calculatePositionBasedOnUltraSonicServos(UltraSonicServo ultraSonicServoX, UltraSonicServo ultraSonicServoY, double robotHeading, Vector2d realsensePos) {
        double rotatedPosX = ultraSonicServoX.getRotatedPosition(robotHeading).getX();
        double rotatedPosY = ultraSonicServoY.getRotatedPosition(robotHeading).getY();

        double deltaXFromRealsense = rotatedPosX - realsensePos.getX();
        double deltaYFromRealsense = rotatedPosY - realsensePos.getY();

        double robotPosX = 24 - ultraSonicServoX.getRawDistance(DistanceUnit.INCH) + deltaXFromRealsense;
        double robotPosY = 72 - ultraSonicServoY.getRawDistance(DistanceUnit.INCH) + deltaYFromRealsense;
        telemetry.addData("DISANCE X: ", ultraSonicServoX.getRawDistance(DistanceUnit.INCH));
        telemetry.addData("DISTNAE Y: ", ultraSonicServoY.getRawDistance(DistanceUnit.INCH));

        return new Pose2d(robotPosY, robotPosX, robotHeading);
    }
}
