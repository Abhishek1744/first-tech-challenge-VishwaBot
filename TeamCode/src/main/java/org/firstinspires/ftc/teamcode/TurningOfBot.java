package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turning Of Bot")
    
public class TurningOfBot extends OpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void init() {
        leftFront  = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack   = hardwareMap.dcMotor.get("leftBack");
        rightBack  = hardwareMap.dcMotor.get("rightBack");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        double drive  = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        leftFront.setPower(drive + strafe + turn);
        rightFront.setPower(drive - strafe - turn);
        leftBack.setPower(drive - strafe + turn);
        rightBack.setPower(drive + strafe - turn);
    }

}
