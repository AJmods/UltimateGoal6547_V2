package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.homar.Button;

@TeleOp
@Disabled
public class RevBlinkenTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RevBlinkinLedDriver lights;

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        telemetry.log().add("Ready to start");
        waitForStart();

        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(0);

        Button a = new Button();
        Button b = new Button();
        while (opModeIsActive()) {
            a.input(gamepad1.a);
            b.input(gamepad1.b);
            if (a.onPress()) {
                pattern = pattern.next();
                lights.setPattern(pattern);
            }
            if (b.onPress()) {
                pattern = pattern.previous();
                lights.setPattern(pattern);
            }
            telemetry.addData("PATTERN NAME: ", pattern.name());
            telemetry.update();
        }
    }
}
