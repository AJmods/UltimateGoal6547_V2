package org.firstinspires.ftc.teamcode.oldFiles.roadRunner.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192; //(8192 Counts per rev) 2048 Cycles per Revolution.  Info got from rev site
    public static double WHEEL_RADIUS = 0.765; // in  //19 mm
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 9.625; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    public static final double COUNTS_PER_INCH = 1743.855179349648;

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(1.625, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(1.625, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(-2, -5.125, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("intake");
        rightEncoder = hardwareMap.dcMotor.get("sideEncoder");
        frontEncoder = hardwareMap.dcMotor.get("vertRight");

        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double encoderTicksToInches(int ticks) {
        return ticks/COUNTS_PER_INCH;
    }



    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }
}
