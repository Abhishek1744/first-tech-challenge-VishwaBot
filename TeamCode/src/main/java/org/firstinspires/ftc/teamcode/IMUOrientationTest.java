package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMU Orientation Test", group = "Test")
public class IMUOrientationTest extends OpMode {

    IMU imu;

    @Override
    public void init() {

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu.initialize(parameters);
    }

    @Override
    public void loop() {

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (deg)", angles.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (deg)", angles.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (deg)", angles.getRoll(AngleUnit.DEGREES));
        telemetry.update();
    }
}
