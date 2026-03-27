package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Laser Distance Test (Analog)", group = "Test")
public class LaserDistanceTest extends OpMode {

    private AnalogInput laser;

    @Override
    public void init() {
        laser = hardwareMap.get(AnalogInput.class, "laserDist");
    }

    @Override
    public void loop() {
        double voltage = laser.getVoltage();
        double distanceMm = (voltage / 3.3) * 1000.0;
        double distanceIn = distanceMm / 25.4;

        telemetry.addData("Voltage", "%.3f V", voltage);
        telemetry.addData("Distance (mm)", "%.1f", distanceMm);
        telemetry.addData("Distance (in)", "%.2f", distanceIn);
        telemetry.update();
    }
}
