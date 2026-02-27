package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic 2 Wheeler Drive", group = "TeleOp")
public class BasicTwoWheelerDrive extends LinearOpMode {

    private static final double DRIVE_SCALE = 1.0;
    private static final double TURN_SCALE = 0.35;
    private static final double STICK_DEADZONE = 0.08;
    private static final boolean REVERSE_RIGHT_BACK = true;

    @Override
    public void runOpMode() {
        DcMotorEx leftFront = getMotor(hardwareMap, "leftFront", "leftfront");
        DcMotorEx rightFront = getMotor(hardwareMap, "rightFront", "rightfront");
        DcMotorEx leftBack = getMotor(hardwareMap, "leftBack", "leftback");
        DcMotorEx rightBack = getMotor(hardwareMap, "rightBack", "rightback");

        if (REVERSE_RIGHT_BACK) {
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Basic 2 Wheeler Ready");
        telemetry.addLine("Driving only REAR motors (front motors fixed at 0)");
        telemetry.addLine("Left Stick Y = Forward/Backward");
        telemetry.addLine("Right Stick X = Turn Left/Right");
        telemetry.addData("Right Rear Reversed", REVERSE_RIGHT_BACK);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Reversed per request: pushing stick forward now commands backward.
            double forward = applyDeadzone(gamepad1.left_stick_y) * DRIVE_SCALE;
            double turn = -applyDeadzone(gamepad1.right_stick_x) * TURN_SCALE;

            double leftPower = Range.clip(forward + turn, -1.0, 1.0);
            double rightPower = Range.clip(forward - turn, -1.0, 1.0);

            // Keep front motors untouched (0 power), drive only rear pair.
            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftBack.setPower(leftPower);
            rightBack.setPower(rightPower);

            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.update();
        }

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }

    private static DcMotorEx getMotor(HardwareMap hw, String... names) {
        RuntimeException last = null;
        for (String name : names) {
            try {
                return hw.get(DcMotorEx.class, name);
            } catch (RuntimeException e) {
                last = e;
            }
        }
        throw last != null ? last : new RuntimeException("Motor not found");
    }
}
