package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name = "Testing 2027", group = "TeleOp")
public class testingold extends LinearOpMode {

    private static final double CLOSE_SERVO_POWER = 1.0;
    private static final double DRIVE_DEADZONE = 0.12;
    private static final double DRIVE_SPEED_SCALE = 1;
    private static final double TURN_SPEED_SCALE = 1;
    private static final double ODO_WHEEL_DIAMETER_MM = 32.0;
    private static final double ODO_TICKS_PER_REV = 8192.0;
    private static final double ODO_MM_PER_TICK =
            (Math.PI * ODO_WHEEL_DIAMETER_MM) / ODO_TICKS_PER_REV;

    @Override
    public void runOpMode() {
        // Four independently driven mecanum wheels.
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        CRServo servo1 = hardwareMap.get(CRServo.class, "Servo1");
        CRServo servo2 = hardwareMap.get(CRServo.class, "Servo2");
        GoBildaPinpointDriver pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint");

        // Front and back motors are mounted in opposite directions on this robot.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // goBILDA Pinpoint with 32 mm, 8192 CPR odometry pods.
        pinpoint.setEncoderResolution(1.0 / ODO_MM_PER_TICK, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        boolean servosRunning = false;
        boolean lastServoButton = false;

        setServoPairPower(servo1, servo2, 0.0);

        telemetry.addLine("Ready");
        telemetry.addLine("Drive: left stick = forward/back and strafe");
        telemetry.addLine("Turn: right stick X = left/right");
        telemetry.addLine("Odometry: Pinpoint X/Y in cm");
        telemetry.addLine("Servo pair: X starts closing; X again stops");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            pinpoint.update();
            double driveInput = applyDeadzone(gamepad1.right_stick_x);
            double strafeInput = applyDeadzone(-gamepad1.left_stick_x);
            double turnInput = applyDeadzone(-gamepad1.left_stick_y);

            double drive = driveInput * DRIVE_SPEED_SCALE;
            double strafe = strafeInput * DRIVE_SPEED_SCALE;
            double turn = turnInput * TURN_SPEED_SCALE;

            // Standard robot-centric mecanum mixing.
            double leftFrontPower = drive + strafe + turn;
            double leftBackPower = drive - strafe + turn;
            double rightFrontPower = drive - strafe - turn;
            double rightBackPower = drive + strafe - turn;

            // Keep every motor within its valid range while preserving direction ratios.
            double denominator = Math.max(1.0, Math.max(Math.abs(leftFrontPower),
                    Math.max(Math.abs(leftBackPower), Math.max(Math.abs(rightFrontPower),
                            Math.abs(rightBackPower)))));
            leftFrontPower /= denominator;
            leftBackPower /= denominator;
            rightFrontPower /= denominator;
            rightBackPower /= denominator;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // X starts continuous rotation in the closing direction (><).
            // The next X press stops both servos.
            if (gamepad1.x && !lastServoButton) {
                servosRunning = !servosRunning;
                setServoPairPower(servo1, servo2,
                        servosRunning ? CLOSE_SERVO_POWER : 0.0);
            }
            lastServoButton = gamepad1.x;

            telemetry.addData("Drive Input", "%.2f", driveInput);
            telemetry.addData("Strafe Input", "%.2f", strafeInput);
            telemetry.addData("Turn Input", "%.2f", turnInput);
            telemetry.addData("Left Front Power", "%.2f", leftFrontPower);
            telemetry.addData("Left Back Power", "%.2f", leftBackPower);
            telemetry.addData("Right Front Power", "%.2f", rightFrontPower);
            telemetry.addData("Right Back Power", "%.2f", rightBackPower);
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                telemetry.addData("X (cm)", "%.2f", pinpoint.getPosX(DistanceUnit.CM));
                telemetry.addData("Y (cm)", "%.2f", pinpoint.getPosY(DistanceUnit.CM));
                telemetry.addData("Heading (deg)", "%.1f",
                        Math.toDegrees(pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)));
            } else {
                telemetry.addData("Odometry", pinpoint.getDeviceStatus());
            }
            telemetry.addData("Servos", servosRunning ? "CLOSING (><)" : "STOPPED");
            telemetry.addData("Servo1 Power", "%.2f", servo1.getPower());
            telemetry.addData("Servo2 Power", "%.2f", servo2.getPower());
            telemetry.update();
        }

        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        setServoPairPower(servo1, servo2, 0.0);
    }

    private void setServoPairPower(CRServo servo1, CRServo servo2, double power) {
        servo1.setPower(power);
        // The two servos face each other, so reverse the second one to close (><).
        servo2.setPower(-power);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < DRIVE_DEADZONE ? 0.0 : value;
    }

    private double clip(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}