
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BioBuzz Record and Return")
public class bioBuzzAutonomous extends LinearOpMode {

    private DcMotor leftFront, leftBack;
    private DcMotor rightFront, rightBack;
    private CRServo servo1, servo2;

    private static final double DRIVE_POWER = 0.45;
    private static final double TURN_POWER = 0.30;
    private static final double SERVO_POWER = 1.0;

    private static final double FORWARD_SECONDS = 5.0;
    private static final double TURN_SECONDS = 0.80;
    private static final double SERVO_SECONDS = 5.0;

    private static final long RECORD_INTERVAL_MS = 20;

    private static class DriveCommand {
        final double forward;
        final double strafe;
        final double turn;
        final long durationMs;

        DriveCommand(double forward, double strafe,
                     double turn, long durationMs) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.durationMs = durationMs;
        }
    }

    private final List<DriveCommand> recordedPath = new ArrayList<>();

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        servo1 = hardwareMap.get(CRServo.class, "Servo1");
        servo2 = hardwareMap.get(CRServo.class, "Servo2");

        // Use the same motor directions as your reference TeleOp.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopDrive();
        setServoPairPower(0);

        telemetry.addLine("BioBuzz Autonomous Ready");
        telemetry.addLine("Press START to run");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        try {
            // 1. Drive forward for 5 seconds and record.
            recordFor(DRIVE_POWER, 0, 0, FORWARD_SECONDS);

            // 2. Turn left and record.
            recordFor(0, 0, -TURN_POWER, TURN_SECONDS);

            stopDrive();

            // 3. Automatically run both servos for 5 seconds.
            ElapsedTime servoTimer = new ElapsedTime();
            setServoPairPower(SERVO_POWER);

            while (opModeIsActive()
                    && servoTimer.seconds() < SERVO_SECONDS) {
                idle();
            }

            setServoPairPower(0);

            if (!opModeIsActive()) return;

            // 4. Replay the recorded drive commands in reverse order.
            for (int i = recordedPath.size() - 1;
                 i >= 0 && opModeIsActive(); i--) {

                DriveCommand cmd = recordedPath.get(i);

                applyDriveCommand(
                        -cmd.forward,
                        -cmd.strafe,
                        -cmd.turn
                );

                sleep(cmd.durationMs);
            }

        } finally {
            stopDrive();
            setServoPairPower(0);
        }

        telemetry.addLine("Autonomous Complete");
        telemetry.update();
    }

    private void recordFor(double forward, double strafe,
                           double turn, double seconds) {

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < seconds) {
            long startMs = System.nanoTime() / 1_000_000L;

            applyDriveCommand(forward, strafe, turn);

            long elapsedMs =
                    System.nanoTime() / 1_000_000L - startMs;

            long durationMs = Math.max(
                    1, RECORD_INTERVAL_MS - elapsedMs);

            recordedPath.add(new DriveCommand(
                    forward, strafe, turn, durationMs));

            sleep(durationMs);
        }

        stopDrive();
    }

    private void applyDriveCommand(double forward,
                                   double strafe,
                                   double turn) {

        double lf = forward + strafe + turn;
        double lb = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rb = forward + strafe - turn;

        double denominator = Math.max(
                1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(lb),
                                Math.max(Math.abs(rf), Math.abs(rb))))
        );

        leftFront.setPower(Range.clip(lf / denominator, -1.0, 1.0));
        leftBack.setPower(Range.clip(lb / denominator, -1.0, 1.0));
        rightFront.setPower(Range.clip(rf / denominator, -1.0, 1.0));
        rightBack.setPower(Range.clip(rb / denominator, -1.0, 1.0));
    }

    private void setServoPairPower(double power) {
        servo1.setPower(power);
        servo2.setPower(-power);
    }

    private void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}