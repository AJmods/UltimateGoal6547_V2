package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Used for the Analog Gyro Sensor
 */
public class AnalogGyroSensor extends AnalogInput {

    public static double VOLTAGE_MULTIPLIER = 1.903030303030303030;

    private double gyroZeroVal = 0;

    /**
     * Constructor
     *
     * @param controller AnalogInput controller this channel is attached to
     * @param channel    channel on the analog input controller
     */
    public AnalogGyroSensor(AnalogInputController controller, int channel) {
        super(controller, channel);
    }

    public void zeroGyro() {
        gyroZeroVal = getVoltage();
    }

    public double getAngle(AngleUnit angleUnit) {
        double angleRAD = norm((getVoltage() - gyroZeroVal)*VOLTAGE_MULTIPLIER);
        if (angleUnit == AngleUnit.DEGREES) return Math.toDegrees(angleRAD);
        else if (angleUnit == AngleUnit.RADIANS) return angleRAD;
        return Double.NaN;
    }
    private double norm(double angle) {
        while (angle > Math.toRadians(360)) angle-=360;
        while (angle > Math.toRadians(0)) angle+=360;
        return angle;
    }
}
