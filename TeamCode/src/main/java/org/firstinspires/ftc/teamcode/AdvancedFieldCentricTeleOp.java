package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Advanced Field Oriented TeleOp")
public class AdvancedFieldCentricTeleOp extends OpMode {

    private MecanumDrive drive;

    // Heading offset for driver reset
    private double headingOffset = 0.0;

    // Tunables
    private static final double TURN_SCALE = 0.8;
    private static final double SLOW_MODE_MULTIPLIER = 0.35;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();

        Pose2d pose = drive.localizer.getPose();

        // --- HEADING (IMU-FUSED) ---
        double heading = pose.heading.toDouble() - headingOffset;

        // --- INPUTS ---
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x * TURN_SCALE;

        // --- HEADING RESET ---
        if (gamepad1.y) {
            headingOffset = pose.heading.toDouble();
        }

        // --- FIELD → ROBOT TRANSFORM ---
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double robotForward = forward * cos + strafe * sin;
        double robotStrafe  = -forward * sin + strafe * cos;

        // --- NORMALIZATION (NO DIAGONAL BOOST) ---
        double max = Math.max(
                Math.abs(robotForward) + Math.abs(robotStrafe) + Math.abs(turn),
                1.0
        );

        robotForward /= max;
        robotStrafe  /= max;
        turn          /= max;

        // --- PRECISION MODE ---
        if (gamepad1.right_bumper) {
            robotForward *= SLOW_MODE_MULTIPLIER;
            robotStrafe  *= SLOW_MODE_MULTIPLIER;
            turn         *= SLOW_MODE_MULTIPLIER;
        }

        // --- APPLY DRIVE POWER ---
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(robotForward, robotStrafe),
                        turn
                )
        );

        // --- TELEMETRY ---
        telemetry.addData("X (in)", pose.position.x);
        telemetry.addData("Y (in)", pose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("Heading Offset (deg)", Math.toDegrees(headingOffset));
        telemetry.update();
    }
}
