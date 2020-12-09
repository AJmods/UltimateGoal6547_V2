package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.localizer.T265LocalizerRR;

@Autonomous
public class stopRealsense extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.log().add("ready to start");
        waitForStart();
        T265LocalizerRR.stopRealsense();
    }
}
