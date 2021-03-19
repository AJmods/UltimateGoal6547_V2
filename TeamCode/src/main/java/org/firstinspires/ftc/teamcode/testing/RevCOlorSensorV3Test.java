package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drivetrain.Bot2;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@TeleOp
@Config
public class RevCOlorSensorV3Test extends LinearOpMode {

    public static double targetPos = 1000;

    private ColorSensor colorSensor;

    Bot2 bot;
    @Override
    public void runOpMode() throws InterruptedException {

        bot = new Bot2(this);

        colorSensor = hardwareMap.get(ColorSensor.class, "color2");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.log().add("ready to start");
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.a) {
               // bot.conveyor.setPower(.3);
                bot.intake();
            } else if (gamepad2.b) {
                bot.conveyor.setPower(-1);
                bot.outtake();
            } else if (!isRingAtConveyor()){
                bot.stopConveyor();
                bot.intake.setPower(0);
            }

            if (isRingAtConveyor() && bot.intake.getPower() >= 0) {
                bot.conveyor.setPower(1);
                bot.intake();
            }

            if (bot.conveyor.getCurrentPosition() > targetPos) {
                bot.stopConveyor();
                zeroConveyor();
            }

            double r = colorSensor.red();
            double g = colorSensor.green();
            double b = colorSensor.blue();
            telemetry.addData("is ring at conveyor", isRingAtConveyor());
            telemetry.addData("Conveyor Pos", bot.conveyor.getCurrentPosition());
            telemetry.addData("R", r);
            telemetry.addData("G", g);
            telemetry.addData("B", b);
            //telemetry.addData("R 2", colorSensor.getNormalizedColors().red);
            telemetry.update();

        }

    }

    public void loadRingInConveyor() {
        bot.conveyor.setPower(1);
    }
    public void stopConveyor() {

    }

    public void zeroConveyor() {
        bot.conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isRingAtConveyor() {
        return colorSensor.red() > 16;
    }
}
