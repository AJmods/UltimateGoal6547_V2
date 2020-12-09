package org.firstinspires.ftc.teamcode.testing.realsense;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class TestT265 extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        RobotLog.d("Started Op Mode");
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(new Translation2d(0,0), new Rotation2d(0)), 0, hardwareMap.appContext);
            RobotLog.d("Created Camera");
        }
//        else
//        {
//            Pose2d pose = slamra.getLastReceivedCameraUpdate().pose;
//            slamra.setPose(new Pose2d(-pose.getTranslation().getX(),-pose.getTranslation().getX(),-pose.getRotation().getDegrees()));
//            RobotLog.d("set SLAMRA pose to 0");
//        }

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        RobotLog.d("Starting");
        slamra.start();
    }

    @Override
    public void loop() {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        packet.addLine("Confidence: " + up.confidence);
        packet.addLine("Tranlaton X:" + translation.getX() +", Y:" + translation.getY());
        packet.addLine("Get Rotation Degrees " + up.pose.getRotation().getDegrees());
        packet.addLine("Get Rotation Heading " + up.pose.getHeading());


        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        slamra.stop();
        RobotLog.d("Stopping Realsese");
    }

}