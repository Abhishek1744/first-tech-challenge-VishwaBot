package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turning Of Bot")
public class TurningOfBot extends OpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        leftFront  = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack   = hardwareMap.dcMotor.get("leftBack");
        rightBack  = hardwareMap.dcMotor.get("rightBack");

        // Reverse right motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        motor.setPower(0.5);
        waitForStart();
        while (opModeIsActive()){

        double drive  = -gamepad1.left_stick_y; // forward/back
        double strafe =  gamepad1.left_stick_x; // left/right

        leftFront.setPower(drive + strafe);
        rightFront.setPower(drive - strafe);
        leftBack.setPower(drive - strafe);
        rightBack.setPower(drive + strafe);
        SetTargetPosition(1000);

    }
}
