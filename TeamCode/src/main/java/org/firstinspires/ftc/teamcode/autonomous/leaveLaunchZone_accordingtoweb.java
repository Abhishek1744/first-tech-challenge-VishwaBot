package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous(name = "Leave Launch Zone", group = "Auto")
public class leaveLaunchZone_accordingtoweb extends LinearOpMode {

    // =======================
    // Hardware & Vision
    // =======================
    private DistanceSensor laserSensor;
    private DcMotor shooterMotor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // =======================
        // Drive initialization
        // =======================
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // =======================
        // Hardware mapping
        // =======================
        laserSensor = hardwareMap.get(DistanceSensor.class, "laser");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");

        // =======================
        // AprilTag setup
        // =======================
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // =======================
        // Road Runner action
        // =======================
        Action leaveLaunchZone = drive.actionBuilder(startPose)
                .lineToY(30)
                .lineToX(60)
                .turn(Math.toRadians(90))
                .build();

        // =======================
        // Start Autonomous
        // =======================
        waitForStart();
        if (isStopRequested()) return;

        // =======================
        // Run movement
        // =======================
        Actions.runBlocking(leaveLaunchZone);

        // =======================
        // Laser safety stop
        // =======================
        double distance = laserSensor.getDistance(DistanceUnit.CM);
        if (distance < 15) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            telemetry.addLine("Obstacle detected! STOPPED");
            telemetry.update();
            return;
        }

        // =======================
        // AprilTag detection
        // =======================
        boolean goalDetected = false;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection tag : detections) {
            if (tag.id == 1) {   // Goal AprilTag ID
                goalDetected = true;
                break;
            }
        }

        // =======================
        // Shoot if detected
        // =======================
        if (goalDetected) {
            shoot();
        }
    }

    // =======================
    // Shooter method
    // =======================
    private void shoot() {
        shooterMotor.setPower(1.0);
        sleep(800);
        shooterMotor.setPower(0);
    }
}
