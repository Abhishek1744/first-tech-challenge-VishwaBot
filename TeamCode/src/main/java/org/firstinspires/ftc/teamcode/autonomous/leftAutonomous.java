package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.autonomous.DistanceSensor;

@Autonomous(name = "Left Autonomous", group = "Autonomous")
public class leftAutonomous extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DistanceSensor DistanceSensor;

    private static final int TURN_45_TIME_MS = 450;
    private static final int TURN_180_TIME_MS = 1800;
    private static final int TURN_90_TIME_MS = 900;
    private static final double SAFE_DISTANCE_CM = 50.0; // Minimum safe distance

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // ✅ Initialize custom distance sensor (FIXED)
        DistanceSensor = new DistanceSensor(hardwareMap, "DistanceSensor");

        // Reverse one side
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Autonomous Ready");
        telemetry.addData("Distance", "%.2f cm", DistanceSensor.getDistanceCM());
        telemetry.update();

        waitForStart();
        startLiveDistanceMonitor();


        if (opModeIsActive()) {

            // Forward with distance check
            driveForwardWithSensor(0.5, 2000);
            stopDrive();
            sleep(300);

            // Strafe right
            strafeRight(0.8);
            sleep(1500);
            stopDrive();
            sleep(300);

            // 45-degree right turn
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
            sleep(1000);
            stopDrive();

            // Forward with distance check
            driveForwardWithConfirm(0.5);
            stopDrive();

            // 90-degree left turn
            turnRight(-0.5);
            sleep(TURN_90_TIME_MS);
            stopDrive();

            // Forward with distance check
            driveForwardWithSensor(0.5, 1000);
            stopDrive();

            // 180-degree right turn
            turnRight(0.5);
            sleep(TURN_180_TIME_MS);
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

    // New method: Drive forward with distance sensor safety check
    private void driveForwardWithSensor(double power, long maxTimeMs) {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < maxTimeMs)) {
            double distance = DistanceSensor.getDistanceCM(); // ✅ FIXED

            telemetry.addData("Distance", "%.2f cm", distance);
            telemetry.update();

            // Stop if obstacle detected within safe distance
            if (distance < SAFE_DISTANCE_CM) {
                telemetry.addLine("Obstacle detected! Stopping.");
                telemetry.update();
            }

            driveForward(power);
            sleep(50); // Small delay for sensor reading
        }
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

    private void turnLeft(double power) {
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    // Utility method to get current distance
    private double getDistanceSensor() {
        return DistanceSensor.getDistanceCM(); // ✅ FIXED
    }

    private void driveForwardWithConfirm(double power) {

        while (opModeIsActive()) {

            double distance = DistanceSensor.getDistanceCM();

            telemetry.addData("Distance", "%.2f cm", distance);
            telemetry.update();

            // If obstacle detected
            if (distance < SAFE_DISTANCE_CM) {

                stopDrive();

                telemetry.addLine("⚠ Obstacle Detected!");
                telemetry.addLine("Turning Right...");
                telemetry.update();

                // Turn right
                turnRight(0.5);
                sleep(450);   // ~45 degrees (tune this)
                stopDrive();

                sleep(300);

                // Continue after turning
            }


            driveForward(power);
            sleep(50);
        }
    }
    // ✅ LIVE DISTANCE FUNCTION
    // ✅ Run live distance in parallel (non-blocking)
    private void startLiveDistanceMonitor() {
        new Thread(() -> {
            while (opModeIsActive()) {
                telemetry.addData("LIVE Distance", "%.2f cm", DistanceSensor.getDistanceCM());
                telemetry.update();
                sleep(50);
            }
        }).start();

    }


}
