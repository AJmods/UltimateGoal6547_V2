package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTuner extends LinearOpMode {

    public static double WOB_GRABBER_POS = 0;
    public static double WOBVATOR_POS = 0;

    private Servo wobbleGrabber;
    private Servo wobvator;
    @Override
    public void runOpMode() throws InterruptedException {

        wobbleGrabber = hardwareMap.get(Servo.class, "wob");
        wobvator = hardwareMap.get(Servo.class, "wobvator");

        telemetry.log().add("Ready to Start");
        waitForStart();

        while (opModeIsActive()) {
            wobbleGrabber.setPosition(WOB_GRABBER_POS);
            wobvator.setPosition(WOBVATOR_POS);
        }

    }
}
