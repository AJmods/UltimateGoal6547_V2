package org.firstinspires.ftc.teamcode.testing.notused;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
@Disabled
@Deprecated
public class WobbleBobble extends LinearOpMode {

    Servo wobbleBobble;

    public static double servoPos = .5;
    @Override
    public void runOpMode() throws InterruptedException {

        wobbleBobble = hardwareMap.get(Servo.class, "wob");
        telemetry.log().add("Ready to Start");

        waitForStart();

        while (opModeIsActive()) {
            wobbleBobble.setPosition(servoPos);
        }



    }
}
