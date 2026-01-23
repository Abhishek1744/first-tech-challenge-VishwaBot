package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Basic Autonomous", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;

    // Tune this value on-field
    private static final int TURN_45_TIME_MS = 450;

    @Override
    public void runOpMode() {

        // Hardware mapping
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Reverse one side
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Forward
            driveForward(0.5);
            sleep(2000);
            stopDrive();

            sleep(300);

            // Strafe right
            strafeRight(0.8);
            sleep(1500);
            stopDrive();

            sleep(300);

            // 🔄 45-degree right turn
            turnRight(0.5);
            sleep(TURN_45_TIME_MS);
            stopDrive();

            sleep(300);

            // Backward
            driveBackward(1.0);
            sleep(200);
            stopDrive();

            sleep(300);

            // Strafe left
            strafeLeft(0.5);
            sleep(3000);
            stopDrive();
        }
    }

    /* ---------- Drive Methods ---------- */

    private void driveForward(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void driveBackward(double power) {
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
    }

    private void strafeRight(double power) {
        leftFront.setPower(power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(power);
    }

    private void strafeLeft(double power) {
        leftFront.setPower(-power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(-power);
    }

    private void turnRight(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
    }

    private void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
