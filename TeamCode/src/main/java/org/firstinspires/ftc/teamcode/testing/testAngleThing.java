package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
@Config
public class testAngleThing extends LinearOpMode {

    AnalogInput angleThing;

    @Override
    public void runOpMode() throws InterruptedException {

        angleThing = hardwareMap.get(AnalogInput.class, "angleThing");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("angle", angleThing.getVoltage());
            telemetry.update();
        }

    }
}
