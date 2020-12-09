package org.firstinspires.ftc.teamcode.oldFiles.roadRunner.localizer;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;
import java.util.List;


/*
    Two Wheel Odometry tracking.
 */
@Config
public class StandardTwowheelLocalizerPhoneGyro extends TwoTrackingWheelLocalizer implements SensorEventListener {
    public static double TICKS_PER_REV = 8192; //(8192 Counts per rev) 2048 Cycles per Revolution.  Info got from rev site
    public static double WHEEL_RADIUS = 0.760; // in  //19 mm
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    //public static double COUNTS_PER_INCH = 1743.855179349648;

    private DcMotor frontEncoder,sideEncoder;

    //variables for gyro operation
    private float zero;
    private float rawGyro;
    public int sensorDataCounter = 0;

    //arrays for gyro operation
    private float[] rotationMatrix = new float[9];
    private float[] orientation = new float[3];
    //objects for gyro operation
    private SensorManager sensorManager;
    private Sensor rotationVectorSensor;

    protected boolean hasBeenZeroed= false;

    float zRotation;

    float lastAngle;
    float currentHead = zRotation;

    public StandardTwowheelLocalizerPhoneGyro(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(1.625, 5, Math.toRadians(0)), //sideEncoder parallel to drive train
                new Pose2d(-2, -5.125, Math.toRadians(90)) // frontEncoder perpendicular to drive train
        ));

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
        lastAngle = currentHead;
        currentHead = zRotation;

        //RobotLog.w("GYRO ANGLE: " + currentHead + ", LAST ANGLE: " + lastAngle);
        if (lastAngle == currentHead) RobotLog.w("SAME ANGLE AS BEFORE PHONE GYRO: " + currentHead);
        return currentHead;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values);
        SensorManager.getOrientation(rotationMatrix, orientation);

        sensorDataCounter++;

        rawGyro = orientation[0]; //this is in RADIANS

        //If the zero hasn't been zeroed do the zero
        if (!hasBeenZeroed) {
            hasBeenZeroed = true;
            zero = rawGyro;
        }
        //Normalize zRotation to be used
        zRotation = normalize2PI(rawGyro - zero);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    protected float normalize2PI(float val) {
        while (val > (2*Math.PI) || val < 0) {

            if (val > (2*Math.PI)) {
                val -= 360;
            }

            if (val < 0) {
                val += 360;
            }
        }
        return val;
    }



}
