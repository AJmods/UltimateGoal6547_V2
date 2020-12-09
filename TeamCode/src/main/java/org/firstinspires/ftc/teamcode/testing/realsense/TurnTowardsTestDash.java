package org.firstinspires.ftc.teamcode.testing.realsense;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.command.PacketAction;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@Config
@Disabled
public class TurnTowardsTestDash extends LinearOpMode {
    public static double startX;
    public static double startY;

    public static double targetX;
    public static double targetY;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense bot = new DriveTrain6547Realsense(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        Pose2d start = new Pose2d(startX, startY);
        Vector2d target = new Vector2d(targetX, targetY);

        double angle = bot.turnTowardsAngle(target, start);

        bot.setPacketAction(new PacketAction() {
            @Override
            public void addToPacket(TelemetryPacket packet, Canvas fieldOverlay) {
                packet.put("angle", angle);
            }
        });
    }
}
