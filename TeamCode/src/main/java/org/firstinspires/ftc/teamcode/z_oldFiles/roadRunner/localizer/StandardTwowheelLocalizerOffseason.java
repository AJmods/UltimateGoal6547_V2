package org.firstinspires.ftc.teamcode.z_oldFiles.roadRunner.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.z_oldFiles.roadRunner.drive.DriveTrain6547Offseason;

import java.util.Arrays;
import java.util.List;


/*
    Two Wheel Odometry tracking.
 */
@Config
public class StandardTwowheelLocalizerOffseason extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192; //(8192 Counts per rev) 2048 Cycles per Revolution.  Info got from rev site
    public static double WHEEL_RADIUS = 0.765; // in  //19 mm
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    //public static double COUNTS_PER_INCH = 1743.855179349648;

    private final DcMotor frontEncoder;
    private final DcMotor sideEncoder;

    private final DriveTrain6547Offseason bot;

    double LastAngle;

    private final List<LynxModule> allHubs;

    public static double forwardX=-1.625;
    public static double forwardY=-5;
    public static double sideX = 2;
    public static double sideY= -5.125;
    public StandardTwowheelLocalizerOffseason(HardwareMap hardwareMap, DriveTrain6547Offseason bot) {
        super(Arrays.asList(
                new Pose2d(forwardX, forwardY, Math.toRadians(0)), //sideEncoder parallel to drive train
                new Pose2d(sideX, sideY, Math.toRadians(90)) // frontEncoder perpendicular to drive train
        ));

        allHubs = hardwareMap.getAll(LynxModule.class);
        this.bot = bot;

        sideEncoder = hardwareMap.dcMotor.get("sideEncoder");
        frontEncoder = hardwareMap.dcMotor.get("vertRight");

        sideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sideEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sideEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
   // public static double encoderTicksToInches(int ticks)
   // {
     //   return ticks/COUNTS_PER_INCH;
//    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(sideEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @Override
    public double getHeading() {
        double currentHead = bot.getRawExternalHeading();

        RobotLog.w("GYRO ANGLE: " + currentHead + ", LAST ANGLE: " + LastAngle);
        if (LastAngle == currentHead) RobotLog.w("SAME ANGLE AS BEFORE: " + currentHead);
        return currentHead;
    }

}
