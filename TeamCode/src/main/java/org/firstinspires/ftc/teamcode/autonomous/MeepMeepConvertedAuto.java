package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Leave Launch Zone (RR + Laser Only)", group = "Auto")
public class MeepMeepConvertedAuto extends LinearOpMode {

    private DistanceSensor laserSensor;

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        laserSensor = hardwareMap.get(DistanceSensor.class, "laser");

        Action leaveLaunchZone = drive.actionBuilder(startPose)
                .lineToY(30)
                .lineToX(60)
                .turn(Math.toRadians(90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(leaveLaunchZone);
        }
    }
