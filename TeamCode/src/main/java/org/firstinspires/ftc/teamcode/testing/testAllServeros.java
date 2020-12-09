package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class testAllServeros extends LinearOpMode {

    Servo wobvator;
    Servo wobGrabber;
    Servo indexer;

    public static double WobvatorServoPos = .5;
    public static double wobGrabberServoPos = .5;
    public static double indexerServePos = .5;

    @Override
    public void runOpMode() throws InterruptedException {

        try {wobvator = hardwareMap.get(Servo.class, "wobvator"); } catch (Exception e) {telemetry.log().add("Wobvator not found");}
        try {wobGrabber = hardwareMap.get(Servo.class, "wob"); } catch (Exception e) {telemetry.log().add("Wobble grabber not found"); }
        try {indexer = hardwareMap.get(Servo.class, "indexer");} catch (Exception e) {telemetry.log().add("indexer not found");}

        telemetry.log().add("Ready to Start");

        waitForStart();

        while (opModeIsActive()) {

            try {wobvator.setPosition(WobvatorServoPos);} catch (Exception ignored) {}
            try {wobGrabber.setPosition(wobGrabberServoPos);} catch (Exception ignored) {}
            try {indexer.setPosition(indexerServePos);} catch (Exception ignored) {}
        }
    }
}
