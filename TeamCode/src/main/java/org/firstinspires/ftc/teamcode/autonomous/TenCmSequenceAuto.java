package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "10cm Sequence Auto", group = "Autonomous")
public class TenCmSequenceAuto extends LinearOpMode {

    // Simple timed method (easy to tune):
    // Tune these values on your robot once.
    private static final double DRIVE_POWER = 0.45;
    private static final double TURN_POWER = 0.20;
    private static final long MOVE_10CM_MS = 1600;   // adjust for true 10 cm
    private static final long MOVE_20CM_MS = MOVE_10CM_MS * 2; // simple timed scale
    private static final long TURN_90_MS = 1400;     // adjust for true 90 deg
    private static final long PAUSE_MS = 150;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Simple Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Your full requested sequence:
        // 1) forward 10cm
        // 2) right turn 90
        // 3) forward 10cm
        // 4) back 10cm
        // 5) left turn 90
        // 6) back 10cm
        moveForward(drive, MOVE_10CM_MS);
        turnRight90(drive);
        moveForward(drive, MOVE_10CM_MS);
        moveBackward(drive, MOVE_10CM_MS);
        turnLeft90(drive);
        moveBackward(drive, MOVE_10CM_MS);

        // 7..12 additional sequence
        runSequence7to12(drive);

        stopDrive(drive);
    }

    private void moveForward(MecanumDrive drive, long ms) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(DRIVE_POWER, 0.0), 0.0));
        sleep(ms);
        stopDrive(drive);
    }

    private void moveBackward(MecanumDrive drive, long ms) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-DRIVE_POWER, 0.0), 0.0));
        sleep(ms);
        stopDrive(drive);
    }

    private void turnRight90(MecanumDrive drive) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), -TURN_POWER));
        sleep(TURN_90_MS);
        stopDrive(drive);
    }

    private void turnLeft90(MecanumDrive drive) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), TURN_POWER));
        sleep(TURN_90_MS);
        stopDrive(drive);
    }

    private void runSequence7to12(MecanumDrive drive) {
        // 7) forward 20 cm
        // 8) right 90 deg
        // 9) forward 10 cm
        // 10) back 10 cm
        // 11) left 90 deg
        // 12) back 20 cm
        moveForward(drive, MOVE_20CM_MS);
        turnRight90(drive);
        moveForward(drive, MOVE_10CM_MS);
        moveBackward(drive, MOVE_10CM_MS);
        turnLeft90(drive);
        moveBackward(drive, MOVE_20CM_MS);
    }

    private void stopDrive(MecanumDrive drive) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        sleep(PAUSE_MS);
    }
}
