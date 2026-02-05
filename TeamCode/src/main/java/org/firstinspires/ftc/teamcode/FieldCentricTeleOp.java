package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Field Centric TeleOp")
public class FieldCentricTeleOp extends OpMode {
    private MecanumDrive drive;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        // Update pose estimate (Pinpoint localizer by default)
        drive.updatePoseEstimate();

        Pose2d pose = drive.localizer.getPose();
        double heading = pose.heading.toDouble();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Rotate field-relative inputs into robot frame
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double robotForward = forward * cos + strafe * sin;
        double robotStrafe = -forward * sin + strafe * cos;

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(robotForward, robotStrafe),
                turn
        ));

        telemetry.addData("x (in)", pose.position.x);
        telemetry.addData("y (in)", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(heading));
        telemetry.update();
    }
}
