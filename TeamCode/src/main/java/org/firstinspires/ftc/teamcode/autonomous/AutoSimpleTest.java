package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "AutoSimpleTest", group = "Test9")
public class AutoSimpleTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // 1️⃣ Define start pose (field origin)
        Pose2d startPose = new Pose2d(0, 0, 0);

        // 2️⃣ Create drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // 3️⃣ Build simple actions
        Action simpleAuto = new SequentialAction(
                drive.actionBuilder(startPose)
                        .lineToX(24)          // move forward ~24 inches
                        .turn(Math.toRadians(90)) // turn 90 degrees CCW
                        .build()
        );
        telemetry.addLine("Init complete");
        telemetry.update();
        // 4️⃣ Wait for start
        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();
        drive.leftFront.setPower(0.8);
        drive.leftBack.setPower(0.8);
        drive.rightBack.setPower(0.8);
        drive.rightFront.setPower(0.8);

        sleep(1000);

        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);


        if (isStopRequested()) return;

        // 5️⃣ Run the action
        Actions.runBlocking(simpleAuto);
    }
}