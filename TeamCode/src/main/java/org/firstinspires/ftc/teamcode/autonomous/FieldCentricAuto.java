package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Field Centric Auto")
public class FieldCentricAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // SAME drive class, SAME localizer
        MecanumDrive drive = new MecanumDrive(
                hardwareMap,
                new Pose2d(0, 0, 0) // start pose
        );

        // Build trajectory in FIELD coordinates
        Action traj = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(24)              // forward 24 inches
                .lineToY(24)              // strafe left 24 inches
                .turn(Math.toRadians(90))// rotate 90 degrees
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Executes while continuously updating odometry
        Actions.runBlocking(traj);
    }
}
