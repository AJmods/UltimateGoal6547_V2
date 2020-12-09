package org.firstinspires.ftc.teamcode.oldFiles.roadRunner.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.oldFiles.roadRunner.drive.DriveTrain6547Offseason;

/*
 * This routine determines the effective track width. The procedure works by executing a point turn
 * with a given angle and measuring the difference between that angle and the actual angle (as
 * indicated by an external IMU/gyro, track wheels, or some other localizer). The quotient
 * given angle / actual angle gives a multiplicative adjustment to the estimated track width
 * (effective track width = estimated track width * given angle / actual angle). The routine repeats
 * this procedure a few times and averages the values for additional accuracy. Note: a relatively
 * accurate track width estimate is important or else the angular constraints will be thrown off.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class AutoTrackWidthTuner extends LinearOpMode {

    public static double MIN_WIDTH = 13;
    public static double MAX_WIDTH = 16;

    private double numToAdd = ((MAX_WIDTH-MIN_WIDTH)/2); //middle

    double currentTrackWidth = numToAdd;


    public static double ANGLE = 180; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DriveTrain6547Offseason drive = new DriveTrain6547Offseason(this);
        // TODO: if you haven't already, set the localizer to something that doesn't depend on
        // drive encoders for computing the heading

        telemetry.addLine("Press play to begin the track width tuner routine");
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            DriveConstants.TRACK_WIDTH = currentTrackWidth;

            drive.turn(Math.toRadians(ANGLE));

            double error=0;

            while (!isStopRequested() && drive.isBusy()) {
                error = drive.getLastError().getHeading();

                drive.update();
            }

            sleep(DELAY);

            numToAdd/=2;
            if (error>0) currentTrackWidth-=numToAdd;
            else if (error<0) currentTrackWidth+=numToAdd;
            telemetry.log().add("Trial " + i + ": Track Width = " + DriveConstants.TRACK_WIDTH + "\nerror: " +error);
        }

        telemetry.clearAll();
        telemetry.addLine("Tuning complete");
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
