package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Single Motor TeleOp")
public class SingleMotorTeleOp extends LinearOpMode {

    private DcMotor singlemotor;

    @Override
    public void runOpMode() {

        singlemotor = hardwareMap.get(DcMotor.class, "singlemotor");

        singlemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y;

            singlemotor.setPower(power);

            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }
}