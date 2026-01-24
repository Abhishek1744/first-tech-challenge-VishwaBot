package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistanceSensor {

    private final AnalogInput sensor;

    private static final double MAX_VOLTAGE = 3.3;
    private static final double MAX_DISTANCE_MM = 500.0;

    public DistanceSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(AnalogInput.class, name);
    }

    public double getVoltage() {
        return sensor.getVoltage();
    }

    public double getDistanceMM() {
        return (getVoltage() / MAX_VOLTAGE) * MAX_DISTANCE_MM;
    }

    public double getDistanceCM() {
        return getDistanceMM() / 10.0;
    }
}