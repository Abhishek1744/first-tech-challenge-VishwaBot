package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBenchIMU {
    private IMU imu;
    private DcMotor motor;

    private void init(HardwareMap hwMap) {
        imu = hwMap.get(IMU.class, "imu");
        motor = hwMap.get(DcMotor.class, "motor");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getHeading(BNO055IMU.AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit)

    }

    public void setMotor(double power) {
        motor.setPower(power);

    }
}
