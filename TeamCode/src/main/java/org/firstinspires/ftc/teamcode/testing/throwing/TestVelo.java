package org.firstinspires.ftc.teamcode.testing.throwing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
@Disabled
public class TestVelo extends LinearOpMode {
    public static double revPerSec = 1;
    public static double minLaucnchAMPS = 3;
    boolean launched = false;
    boolean stateChanged = false;
    DcMotorEx motor;
    DcMotorEx motor2;

    ElapsedTime time = new ElapsedTime();
    double lastTick = 0;

    final double dm = 3.77953; //96 mm

    public double vTicks = -11.89*2.36*46.6666;
    //public double thing = v / (Math.PI * dm);

    public static double DrewsConstant = 1450/487.250;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        motor = hardwareMap.get(DcMotorEx.class, "thrower");
        motor2 = hardwareMap.get(DcMotorEx.class, "thrower1");

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        time.reset();

        waitForStart();

        while (opModeIsActive()) {

            motor.setVelocity(revPerSec*360, AngleUnit.DEGREES);
            motor2.setVelocity(revPerSec*360, AngleUnit.DEGREES);

            launched = motor.getCurrent(CurrentUnit.AMPS) > minLaucnchAMPS;

            telemetry.addData("LAUNCHED", launched);
            telemetry.addData("Target V: ", vTicks);
            telemetry.addData("curernt V (guess)", getVTicksPerSec(motor));
            telemetry.addData("current V (Degrees)", motor.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("current V (Radains)", motor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Current V (ticks)", motor.getVelocity());
            telemetry.addData("MOTOR 1 AMPS:", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("MOTOR 2 AMPS:", motor2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current POW", motor.getPower());
            telemetry.update();
        }


    }

    public double getVTicksPerSec(DcMotorEx motor) {
        double timeBetweenIntervel = time.seconds();
        double currentTicks = motor.getCurrentPosition() - lastTick;

        double answer = currentTicks / timeBetweenIntervel;

        time.reset();
        lastTick = motor.getCurrentPosition();
        return answer;
    }

    public static double getVi(double x1, double y1, double x2, double y2, double angle) {

        double gravity = -386.09;
        int numDigits = 10;
        double startNumber = 1000;

        double deltaX = Math.abs(x1 - x2);
        double deltaY = Math.abs(y2 - y1);
        angle = Math.toRadians(angle);

        double answer = deltaX / (Math.cos(angle) * Math.sqrt((deltaY - (deltaX * Math.tan(angle))) / (gravity / 2)));
        return answer;
    }
}
