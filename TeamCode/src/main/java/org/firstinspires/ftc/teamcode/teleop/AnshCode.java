
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Ansh_Code", group = "TeleOp")
public class AnshCode extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    @Override
    public void runOpMode() {

        // Initialize motors using your configuration names
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Motor directions (preserved from your code)
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Stop all motors during initialization
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        telemetry.addLine("Basic TeleOp Initialized");
        telemetry.addLine("Left stick: Move and strafe");
        telemetry.addLine("Right stick X: Turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // LEFT STICK: Forward / Backward (unchanged)
            double y = -gamepad1.left_stick_x;

            // LEFT STICK: Strafe Left / Right (unchanged)
            double x = gamepad1.left_stick_y;

            // RIGHT STICK: Turn Left / Right (unchanged)
            double turn = gamepad1.right_stick_x;

            // Joystick deadzone
            double deadzone = 0.05;

            if (Math.abs(y) < deadzone) y = 0;
            if (Math.abs(x) < deadzone) x = 0;
            if (Math.abs(turn) < deadzone) turn = 0;

            double lf, lb, rf, rb;

            if (turn > 0) {

                // RIGHT TURN
                // Left motors forward, right motors backward
                lf = turn;
                lb = turn;
                rf = -turn;
                rb = -turn;

            } else if (turn < 0) {

                // LEFT TURN
                // Left motors backward, right motors forward
                lf = turn;
                lb = turn;
                rf = -turn;
                rb = -turn;

            } else {

                // Normal forward/backward and strafing
                lf = y + x;
                lb = y - x;
                rf = y - x;
                rb = y + x;
            }

            // Normalize powers to keep motors within -1 to +1
            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.max(Math.abs(lf), Math.abs(lb)),
                            Math.max(Math.abs(rf), Math.abs(rb))
                    )
            );

            lf /= max;
            lb /= max;
            rf /= max;
            rb /= max;

            // Apply power to motors
            leftFront.setPower(lf);
            leftBack.setPower(lb);
            rightFront.setPower(rf);
            rightBack.setPower(rb);

            // Driver Station telemetry
            telemetry.addData("Forward", "%.2f", y);
            telemetry.addData("Strafe", "%.2f", x);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("LF", "%.2f", lf);
            telemetry.addData("LB", "%.2f", lb);
            telemetry.addData("RF", "%.2f", rf);
            telemetry.addData("RB", "%.2f", rb);
            telemetry.update();
        }
    }
}