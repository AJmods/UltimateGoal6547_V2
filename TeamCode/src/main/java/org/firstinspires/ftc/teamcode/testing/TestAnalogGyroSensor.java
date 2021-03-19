package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.util.AnalogGyroSensor;

public class TestAnalogGyroSensor extends LinearOpMode {

    DriveTrain6547Realsense bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new DriveTrain6547Realsense(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Angle: ", bot.getAnalogGyroSensor().getAngle(AngleUnit.DEGREES));
        }
    }
}
