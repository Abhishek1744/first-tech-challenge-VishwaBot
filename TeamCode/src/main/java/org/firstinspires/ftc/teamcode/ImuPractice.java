package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ImuPractice extends OpMode {
    TestBenchIMU bench = new TestBenchIMU();
    double heading;


    @Override
    public void init() {
        bench.init(hardwareMap);

    }

    @Override
    public void loop() {
        heading = bench.getHeading(BNO055IMU.AngleUnit.DEGREES);
        telemetry.addData("Heading", bench.getHeading(BNO055IMU.AngleUnit.RADIANS));

        if (heading < 0.5 && heading > -0.5) {
            bench.setMotor(0.0);

        }
        else if (heading > 0.5) {
            bench.setMotor(0.5);
        }
        else {
            bench.setMotor(-0.5);
        }
    }

}
