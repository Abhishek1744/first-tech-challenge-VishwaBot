package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Basic Odometry TeleOp")
public class BasicOdometryTeleOp extends OpMode {

    RobotHardwareEssentials robot;

    @Override
    public void init() {
        robot = new RobotHardwareEssentials(hardwareMap);
    }

    @Override
    public void loop() {

        double drive  = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double turn   =  gamepad1.right_stick_x;

        robot.setDrivePower(drive, strafe, turn);

        robot.updateOdometry();

        telemetry.addData("X (cm)", robot.pos.x);
        telemetry.addData("Y (cm)", robot.pos.y);
        telemetry.addData("Heading (rad)", robot.pos.h);
        telemetry.update();
    }
}