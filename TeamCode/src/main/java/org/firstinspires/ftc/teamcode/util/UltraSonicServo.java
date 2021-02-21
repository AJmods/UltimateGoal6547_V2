package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class UltraSonicServo {

    Servo servo;
    AnalogInput distanceSensor;

    private double posAtLowAngle = 0;
    private double posAtHighAngle = .4;

    private double lowAngle = 90;
    private double highAngle = 0;

    private Pose2d sensorPos;

    private double lastVoltage = 0;
    public UltraSonicServo(Servo servo, AnalogInput distanceSensor, Pose2d sensorPos) {
        this.servo = servo;
        this.distanceSensor = distanceSensor;
        this.sensorPos = sensorPos;
    }

    public UltraSonicServo(Servo servo, AnalogInput distanceSensor) {
        this.servo = servo;
        this.distanceSensor = distanceSensor;
        sensorPos = new Pose2d();
    }

    public void adjustServoBasedOnRotation(double robotAngle) {
        double ratio = Math.abs((lowAngle - robotAngle)/(highAngle - lowAngle));
        double range = posAtHighAngle - posAtLowAngle;
        double servoPos = (ratio * range) + posAtLowAngle;
        servo.setPosition(servoPos);
    }

    public Pose2d getRotatedPosition(double angleToRotateBy, AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.RADIANS) return getRotatedPosition(angleToRotateBy);
        if (angleUnit == AngleUnit.DEGREES) return getRotatedPosition(Math.toRadians(angleToRotateBy));
        return new Pose2d();

    }

    /**
     * @param angleToRotateBy in RADIANS
     * @return position of servo rotated from the origin.
     */
    public Pose2d getRotatedPosition(double angleToRotateBy) {
        double newX = sensorPos.getX()*Math.cos(angleToRotateBy) - sensorPos.getY()*Math.sin(angleToRotateBy);
        double newY = sensorPos.getX()*Math.sin(angleToRotateBy) + sensorPos.getY()*Math.cos(angleToRotateBy);

        return new Pose2d(newX, newY, sensorPos.getHeading());
    }

    /**
     * @param distanceUnit
     * @return the distance that the Maxbotix Ultrasonic Sensor reads
     */
    public double getRawDistance(DistanceUnit distanceUnit) {
        double rawVoltage = distanceSensor.getVoltage();

        if (rawVoltage > 1.4823) { //1.4823 volts is about 144 inches, the width of the field.  If the volate is greater then this, there must have been a faulty reading
            rawVoltage = lastVoltage; //use last distance instead of new one.
        } else {
            lastVoltage = rawVoltage;
        }

        //Constants gotten from getting the true distance from distance sensor to wall via tape measure,
        //then (distance/voltage) got the values in inches, then converted that to other distance units
        if (distanceUnit == DistanceUnit.CM) return rawVoltage*  542.1822921180930552;
        if (distanceUnit == DistanceUnit.METER) return rawVoltage * 5.421822921180930552;
        if (distanceUnit == DistanceUnit.MM) return rawVoltage * 5421.822921180930552;
        if (distanceUnit == DistanceUnit.INCH) return rawVoltage * 213.45759532208388;
        return Double.NaN;
    }


    public Servo getServo() {
        return servo;
    }

    public AnalogInput getDistanceSensor() {
        return distanceSensor;
    }

    public double getPosAtLowAngle() {
        return posAtLowAngle;
    }

    public void setPosAtLowAngle(double posAtLowAngle) {
        this.posAtLowAngle = posAtLowAngle;
    }

    public double getPosAtHighAngle() {
        return posAtHighAngle;
    }

    public void setPosAtHighAngle(double posAtHighAngle) {
        this.posAtHighAngle = posAtHighAngle;
    }

    public double getLowAngle() {
        return lowAngle;
    }

    public void setLowAngle(double lowAngle) {
        this.lowAngle = lowAngle;
    }

    public double getHighAngle() {
        return highAngle;
    }

    public void setHighAngle(double highAngle) {
        this.highAngle = highAngle;
    }
}
