package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Example route: forward 100 cm, turn right 90 degrees, forward 50 cm, stop.
 */
@Autonomous(name = "Two Wheel Encoder Auto", group = "Autonomous")
public class TwoWheelEncoderAuto extends LinearOpMode {

    // Tune these two values if the measured distances or turns are inaccurate.
    // 537.7 is the encoder count per motor revolution for a goBILDA 5202 312 RPM motor.
    private static final double TICKS_PER_MOTOR_REV = 537.7;
    private static final double WHEEL_DIAMETER_CM = 9.0;
    private static final double TRACK_WIDTH_CM = 30.0; // Center-to-center wheel distance.

    private static final double DRIVE_POWER = 0.55;
    private static final double TURN_POWER = 0.40;
    private static final double TIMEOUT_SECONDS = 10.0;

    private static final double TICKS_PER_CM =
            TICKS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_CM);

    @Override
    public void runOpMode() {
        DcMotor leftWheel = hardwareMap.get(DcMotor.class, "LeftW");
        DcMotor rightWheel = hardwareMap.get(DcMotor.class, "RightW");

        // Same direction configuration as KripaBot TeleOp.
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders(leftWheel, rightWheel);

        telemetry.addLine("Encoder auto ready");
        telemetry.addLine("Route: forward 100 cm, right 90 deg, forward 50 cm");
        telemetry.addData("Ticks per cm", "%.2f", TICKS_PER_CM);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        driveCm(leftWheel, rightWheel, 100.0, -DRIVE_POWER);
        turnDegrees(leftWheel, rightWheel, 90.0, -TURN_POWER);
        driveCm(leftWheel, rightWheel, 50.0, -DRIVE_POWER);

        stopAndUseEncoders(leftWheel, rightWheel);
        telemetry.addLine("Autonomous complete");
        telemetry.update();
    }

    private void driveCm(DcMotor leftWheel, DcMotor rightWheel, double cm, double power) {
        int ticks = (int) Math.round(cm * TICKS_PER_CM);
        runToPosition(leftWheel, rightWheel, ticks, ticks, power, "Drive " + cm + " cm");
    }

    private void turnDegrees(DcMotor leftWheel, DcMotor rightWheel, double degrees, double power) {
        // Each wheel travels an arc equal to half the track width during an in-place turn.
        double wheelTravelCm = Math.PI * TRACK_WIDTH_CM * degrees / 360.0;
        int ticks = (int) Math.round(wheelTravelCm * TICKS_PER_CM);
        runToPosition(leftWheel, rightWheel, ticks, -ticks, power, "Right turn " + degrees + " deg");
    }

    private void runToPosition(DcMotor leftWheel, DcMotor rightWheel,
                               int leftDeltaTicks, int rightDeltaTicks,
                               double power, String step) {
        leftWheel.setTargetPosition(leftWheel.getCurrentPosition() + leftDeltaTicks);
        rightWheel.setTargetPosition(rightWheel.getCurrentPosition() + rightDeltaTicks);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setPower(Math.abs(power));
        rightWheel.setPower(Math.abs(power));

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && (leftWheel.isBusy() || rightWheel.isBusy())
                && timer.seconds() < TIMEOUT_SECONDS) {
            telemetry.addData("Step", step);
            telemetry.addData("Left encoder", "%d / %d", leftWheel.getCurrentPosition(), leftWheel.getTargetPosition());
            telemetry.addData("Right encoder", "%d / %d", rightWheel.getCurrentPosition(), rightWheel.getTargetPosition());
            telemetry.update();
            idle();
        }

        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
    }

    private void resetEncoders(DcMotor leftWheel, DcMotor rightWheel) {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stopAndUseEncoders(leftWheel, rightWheel);
    }

    private void stopAndUseEncoders(DcMotor leftWheel, DcMotor rightWheel) {
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
