package org.firstinspires.ftc.teamcode.testing.throwing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.ThrowerUtil;

@TeleOp
@Config
@Disabled
public class TestThowerDisplay extends LinearOpMode {

    public static double REV_PER_SEC = 0;

    public static double x1 = 0;
    public static double x2 = 73;
    public static double y1 = 13;
    public static double y2 = 36;
    public static double angle = 37.5;

    double maxFeet = 60;  //5 feet
    double maxLaunchLengthX = 144; //12 feet

    final double dm = 3.77953; //96 mm
    final double cir = Math.PI * dm;

    double lastTick = 0;

    DcMotorEx motor;
    DcMotorEx motor2;

    ElapsedTime time = new ElapsedTime();

    public static double DrewsConstant = 1225/487.250;

    public static double leewayRev = 2;

    public static boolean isRingLaunched = false;
    public static boolean detectRingLaunch = true;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    double endTime;
    double vx;
    double vy;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotorEx.class, "thrower");
        motor2 = hardwareMap.get(DcMotorEx.class, "thrower1");

        telemetry.log().add("Ready to start");
        waitForStart();

        time.reset();
        while (opModeIsActive()) {

            double vi = getVi(x1, y1, x2, y2, angle);

            double vTicks = (vi * 28)/ (Math.PI * dm);

//            double angV = vi / dm / 2;
//            double tickPerSec = angV * (28 / cir);

            motor.setVelocity(REV_PER_SEC*360, AngleUnit.DEGREES);
            motor2.setVelocity(REV_PER_SEC*360, AngleUnit.DEGREES);

            if (detectRingLaunch) {
                double xi = 0;
                double yi = 0;

                double[] revPerSec = new double[] {motor.getVelocity(AngleUnit.DEGREES)*360, motor2.getVelocity(AngleUnit.DEGREES)*360};

                boolean isMotor1TargetVelo = revPerSec[0] > REV_PER_SEC-leewayRev && revPerSec[0] < REV_PER_SEC + leewayRev;
                boolean isMotor2TargetVelo = revPerSec[1] > REV_PER_SEC-leewayRev && revPerSec[1] < REV_PER_SEC + leewayRev;

                double hVelo = vi*Math.cos(Math.toRadians(angle));

                if ((!isMotor1TargetVelo || !isMotor2TargetVelo) && !isRingLaunched) {
                    isRingLaunched = true;
                    time.reset();
                    endTime = (x2-x1)/hVelo;
                } else if (isRingLaunched && time.seconds() < endTime) {
                    vx = hVelo * Math.cos(Math.toRadians(angle));
                    vy = hVelo * Math.sin(Math.toRadians(angle));

                    double currentRingX = (vx*time.seconds()) + xi;
                    double currentRingY = (vy*time.seconds()) + yi;
                  //  Vector2d ringPos = new Vector2d(currentRingX, currentRingY);

                    TelemetryPacket packet = new TelemetryPacket();
                    Canvas fieldOverlay = packet.fieldOverlay();

                    fieldOverlay.setFill("#FF9F21");
                    fieldOverlay.strokeCircle(currentRingX, currentRingY, 2);

                    dashboard.sendTelemetryPacket(packet);

                } else isRingLaunched = false;



            }
            //motor2.setVelocity(vTicks * DrewsConstant*2);

            telemetry.addData("Vi (Rev/s)", vi/ ThrowerUtil.inchesPerRev);
            telemetry.addData("Current REV/calculated Rev ", REV_PER_SEC / (vi/ ThrowerUtil.inchesPerRev));
            telemetry.addData("current Motor 1 V (REV/s)", motor.getVelocity(AngleUnit.DEGREES)/360);
            telemetry.addData("current Motor 2 V (REV/s)", motor2.getVelocity(AngleUnit.DEGREES)/360);
            telemetry.addData("Current Motor 1 POW", motor.getPower());
            telemetry.addData("Current Motor 2 POW", motor2.getPower());
            telemetry.addData("Current Motor 1 AMPS", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current Motor 1 CurrentAlert", motor.getCurrentAlert(CurrentUnit.AMPS));
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
