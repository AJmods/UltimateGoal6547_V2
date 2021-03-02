package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode._teleOp.LeagueChampionshipTeleop;

@TeleOp
@Config
public class testAnalogGyro extends LinearOpMode {

    AnalogInput gyroSensor;
    AnalogInput gyroSensor2;

    public static double MULTIPLER = 1.903030303030303030;
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime gyroTime = new ElapsedTime();

    double lastVolt = 0;
    double lastGyroUpdateTime = 0;

    double gyroZeroVal = 0;

    DriveTrain6547Realsense bot;
    LeagueChampionshipTeleop leagueChampionshipTeleop;
    @Override
    public void runOpMode() throws InterruptedException {

        bot = new DriveTrain6547Realsense(this);
        leagueChampionshipTeleop = new LeagueChampionshipTeleop(this, bot);
        gyroSensor = hardwareMap.get(AnalogInput.class, "gyro");
        gyroSensor2 =  hardwareMap.get(AnalogInput.class, "gyro2");
       // MULTIPLER = gyroSensor.getMaxVoltage()/(2*Math.PI);

        zeroGyro(gyroSensor);

        telemetry.log().add(gyroSensor.getConnectionInfo());
        telemetry.log().add("MAX VOLTAGE", gyroSensor.getMaxVoltage());
        telemetry.log().add("Ready to Start");
        waitForStart();

        elapsedTime.reset();
        gyroTime.reset();
        while (opModeIsActive()) {
            leagueChampionshipTeleop.doTeleOp();
            double gyroVolt = gyroSensor.getVoltage();
            if (gyroVolt != lastVolt) {
                lastVolt = gyroVolt;
                lastGyroUpdateTime = gyroTime.milliseconds();
                gyroTime.reset();
            }
            telemetry.addData("Gyro Volts", gyroSensor.getVoltage());
            telemetry.addData("Gyro 2 volts", gyroSensor2.getVoltage());
            telemetry.addData("Predicted Angle (Analog) (DEG)", getAngle(gyroSensor2, AngleUnit.DEGREES));
            telemetry.addData("Predicted Angle 2 (Analog) (DEG)", getAngle(gyroSensor, AngleUnit.DEGREES));
            telemetry.addData("Actual Angle (Realsense) (DEG)", Math.toDegrees(bot.getPoseEstimate().getHeading()));
            telemetry.addData("Real Angle / voltage", Math.toDegrees(bot.getPoseEstimate().getHeading()) / gyroSensor.getVoltage());
            telemetry.addData("Last Gyro Update time", lastGyroUpdateTime);
            telemetry.addData("update time: ", elapsedTime.milliseconds());
            telemetry.update();
            elapsedTime.reset();
            bot.update();
        }
    }
    public double getAngle(AnalogInput gyroSensor, AngleUnit angleUnit) {
        double angleRAD = (gyroSensor.getVoltage() - gyroZeroVal)*MULTIPLER;
        if (angleUnit == AngleUnit.DEGREES) return Math.toDegrees(angleRAD);
        else if (angleUnit == AngleUnit.RADIANS) return angleRAD;
        return Double.NaN;
    }
    public void zeroGyro(AnalogInput gyroSensor) {
        gyroZeroVal = gyroSensor.getVoltage();
    }
}
