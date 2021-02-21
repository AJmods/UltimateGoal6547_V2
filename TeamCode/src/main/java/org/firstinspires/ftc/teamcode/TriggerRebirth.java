package org.firstinspires.ftc.teamcode;

import com.jakewharton.processphoenix.ProcessPhoenix;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * Tries (and fails) to restart the robot
 */
@Autonomous(name = "Trigger Rebirth")
@Disabled
public class TriggerRebirth  extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.log().add("Press play to trigger the rebirth of the robot app");
        telemetry.log().add("(This will reset the realsense angle)");
        waitForStart();
        //if (Math.random() < .05) playCrypticMessage();
        telemetry.log().add("Triggering Rebirth.....");

        if (!isStopRequested())
        ProcessPhoenix.triggerRebirth(AppUtil.getDefContext());

    }

}
