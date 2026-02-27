package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Motor Direction Test", group = "Autonomous")
public class MotorDirectionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, null);

        telemetry.addLine("Ready - will drive all motors forward for 1s");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < 1.0) {
            drive.leftFront.setPower(-0.8);
            drive.leftBack.setPower(-0.8);
            drive.rightBack.setPower(-0.8);
            drive.rightFront.setPower(-0.8);
        }

        drive.leftFront.setPower(0.0);
        drive.leftBack.setPower(0.0);
        drive.rightBack.setPower(0.0);
        drive.rightFront.setPower(0.0);

        while (opModeIsActive() && timer.seconds() < 1.0) {
            drive.leftFront.setPower(0.8);
            drive.leftBack.setPower(0.8);
            drive.rightBack.setPower(-0.8);
            drive.rightFront.setPower(-0.8);
        }
    }
}
