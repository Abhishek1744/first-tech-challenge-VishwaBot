package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "KripaBot", group = "TeleOp")
@Disabled
public class kripaBot extends LinearOpMode {

        private static final double DRIVE_DEADZONE = 0.12;
        private static final double DRIVE_SPEED_SCALE = 0.5;
        private static final double TURN_SPEED_SCALE = 1.0;

        @Override
        public void runOpMode() {

            // Get the two drive motors
            DcMotor leftWheel = hardwareMap.get(DcMotor.class, "LeftW");
            DcMotor rightWheel = hardwareMap.get(DcMotor.class, "RightW");

            // Reverse the left motor so both wheels move correctly
            leftWheel.setDirection(DcMotor.Direction.REVERSE);

            // Brake when joystick is released
            leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addLine("Two Wheel Drive Ready");
            telemetry.addLine("Left Stick Y = Forward / Reverse");
            telemetry.addLine("Left Stick X = Turn");
            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {

                // Forward / backward
                double driveInput = applyDeadzone(-gamepad1.left_stick_y);

                // Turning
                double turnInput = applyDeadzone(-gamepad1.right_stick_x);

                double drive = driveInput * DRIVE_SPEED_SCALE;
                double turn = turnInput * TURN_SPEED_SCALE;

                // Calculate motor power
                double leftPower = Range.clip(drive + turn, -1.0, 1.0);
                double rightPower = Range.clip(drive - turn, -1.0, 1.0);

                // Set motor power
                leftWheel.setPower(leftPower);
                rightWheel.setPower(rightPower);

                // Telemetry
                telemetry.addData("Drive", "%.2f", drive);
                telemetry.addData("Turn", "%.2f", turn);
                telemetry.addData("Left Wheel", "%.2f", leftPower);
                telemetry.addData("Right Wheel", "%.2f", rightPower);
                telemetry.update();
            }

            // Stop motors
            leftWheel.setPower(0);
            rightWheel.setPower(0);
        }

        private double applyDeadzone(double value) {
            return Math.abs(value) < DRIVE_DEADZONE ? 0.0 : value;
        }
}
