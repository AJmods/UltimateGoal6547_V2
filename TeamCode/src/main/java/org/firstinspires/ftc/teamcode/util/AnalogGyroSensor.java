package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
        double angleRAD = (getVoltage() - gyroZeroVal)*VOLTAGE_MULTIPLIER;
        if (angleUnit == AngleUnit.DEGREES) return Math.toDegrees(angleRAD);
        else if (angleUnit == AngleUnit.RADIANS) return angleRAD;
        return Double.NaN;
    }
}
