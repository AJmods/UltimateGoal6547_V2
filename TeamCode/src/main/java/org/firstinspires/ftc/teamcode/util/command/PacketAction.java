package org.firstinspires.ftc.teamcode.util.command;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public interface PacketAction {

    void addToPacket(TelemetryPacket packet, Canvas fieldOverlay);
}
