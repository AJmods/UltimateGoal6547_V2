package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.localizer.T265LocalizerRR;

/**
 * Stops realsense.  You don't need to use this.
 */
@Autonomous
@Disabled
public class stopRealsense extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.log().add("ready to start");
        waitForStart();
        T265LocalizerRR.stopRealsense();
    }
}
