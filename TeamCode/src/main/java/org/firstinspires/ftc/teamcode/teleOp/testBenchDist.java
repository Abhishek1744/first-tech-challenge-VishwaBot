package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class testBenchDist {

    private DistanceSensor distance;

    public void init(HardwareMap hwMap) {
        distance = hwMap.get(DistanceSensor.class, "sensor_distance");
    }

    public double getDistanceInches() {

        if (distance == null) {
            return -1;
        }

        double reading = distance.getDistance(DistanceUnit.INCH);

        // Handle invalid sensor values
        if (Double.isNaN(reading) || Double.isInfinite(reading)) {
            return -1;
        }

        return reading;
    }
}
