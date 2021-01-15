package org.firstinspires.ftc.teamcode.util.command;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * used to add stuff in the packet that is called when the bot updates in update()
 * @see org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense
 */
public interface PacketAction {

    void addToPacket(TelemetryPacket packet, Canvas fieldOverlay);
}
