package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Basic Autonomous", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;

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

            // Move forward
            setPower(0.5);
            sleep(2000);

            // Stop
            setPower(0);
        }
    }

    private void setPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
}


//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Autonomous(name = "Basic Autonomous", group = "Autonomous")
//public class BasicAutonomous extends LinearOpMode {
//
//    DcMotor leftFront, leftBack, rightFront, rightBack;
//
//    @Override
//    public void runOpMode() {
//
//        // Hardware mapping
//        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
//        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
//        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
//
//        // Reverse one side (depends on your build)
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.setDirection(DcMotor.Direction.REVERSE);
//
//        // Brake when power = 0
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addLine("Autonomous Ready");
//        telemetry.update();
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//
//            // Move forward
//            setPower(0.5);
//            sleep(2000);
//
//            // Stop
//            setPower(0);
//        }
//    }
//
//    private void setPower(double power) {
//        leftFront.setPower(power);
//        leftBack.setPower(power);
//        rightFront.setPower(power);
//        rightBack.setPower(power);
//    }
//}