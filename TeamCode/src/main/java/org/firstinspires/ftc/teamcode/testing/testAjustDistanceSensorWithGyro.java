package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;

@TeleOp
@Config
@Disabled
public class testAjustDistanceSensorWithGyro extends LinearOpMode {

    DriveTrain6547Realsense bot;

    //Rev2mDistanceSensor distanceSensor;
    AnalogInput distanceSensor;

    Servo ultrasonicServo;

    public static double SERVO_POS_AT_0 = .55;
    public static double SERVO_POS_AT_90 = .95;

    public static double Angle90 = Math.toRadians(90);
    public static double Angle0 = 0;

    public static double currentRobotDeg = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new DriveTrain6547Realsense(this);

        distanceSensor = hardwareMap.get(AnalogInput.class, "distance0");
        ultrasonicServo = hardwareMap.get(Servo.class, "dServo");

        telemetry.log().add("ready to start");
        waitForStart();

            while (opModeIsActive()) {
                ajustUltraSonicSensorByGyro(ultrasonicServo);
                Pose2d pos = bot.getPoseEstimate();
                telemetry.addData("Robot POS", pos.toString());
                telemetry.addData("Servo pos", ultrasonicServo.getPosition());
                telemetry.addData("Distance Sensor Voltage",distanceSensor.getVoltage());
                telemetry.addData("Distance Sensor Inches guess", distanceSensor.getVoltage() * 500 / 2.54);
                telemetry.addData("Connection info",distanceSensor.getConnectionInfo());
                telemetry.update();

                if (pos.getHeading() > currentRobotDeg + Math.toRadians(2) || pos.getHeading() < currentRobotDeg - Math.toRadians(2) && bot.mode == DriveTrain6547Realsense.Mode.IDLE) bot.turnRelativeSync(Math.toRadians(currentRobotDeg));
                bot.update();
            }
    }

    public void ajustUltraSonicSensorByGyro(Servo servo) {
        Pose2d pos = bot.getPoseEstimate();
        double ratio = Math.abs((Angle0 - pos.getHeading())/(Angle90 - Angle0));
        double range = SERVO_POS_AT_90 - SERVO_POS_AT_0;
        double servoPos = (ratio * range) + SERVO_POS_AT_0;
        servo.setPosition(servoPos);
    }
}
