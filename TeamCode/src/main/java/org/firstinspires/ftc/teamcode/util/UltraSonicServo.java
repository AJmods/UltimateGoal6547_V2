package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class UltraSonicServo {

    Servo servo;
    AnalogInput distanceSensor;

    private double posAtLowAngle = .55;
    private double posAtHighAngle = .95;

    private double lowAngle = 90;
    private double highAngle = 0;

    private Pose2d sensorPos;

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
        //according to the Maxbotix Website, multiping the voltage by 1250 converts the voltage to CM
        //All the other values were derived from this number.
        //https://www.maxbotix.com/firstrobotics
        if (distanceUnit == DistanceUnit.CM) return rawVoltage*  1250;
        if (distanceUnit == DistanceUnit.METER) return rawVoltage * 12.5;
        if (distanceUnit == DistanceUnit.MM) return rawVoltage * 12500;
        if (distanceUnit == DistanceUnit.INCH) return rawVoltage * 492.126;
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
