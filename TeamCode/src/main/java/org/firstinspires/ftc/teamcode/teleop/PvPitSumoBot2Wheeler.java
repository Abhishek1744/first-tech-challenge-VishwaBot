package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Pvpit Sumo Bot 2 Wheeler", group = "TeleOp")
public class PvPitSumoBot2Wheeler extends LinearOpMode {

    private static final double POSITION_KP = 0.04;
    private static final double HEADING_KP = 1.20;

    private static final double MAX_AUTO_FORWARD = 0.22;
    private static final double MAX_AUTO_TURN = 0.28;
    private static final double MIN_AUTO_FORWARD = 0.04;
    private static final double DRIVE_HEADING_WINDOW_RAD = Math.toRadians(12.0);

    private static final double POSITION_TOLERANCE_INCH = 2.0;
    private static final int STABLE_LOOP_COUNT_REQUIRED = 3;

    private static final double TARGET_X = 0.0;
    private static final double TARGET_Y = 0.0;

    private static final double MANUAL_SPEED_SCALE = 1.00;
    private static final double STICK_DEADZONE = 0.08;
    private static final int MAX_RECORDED_SAMPLES = 6000;

    @Override
    public void runOpMode() {
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

        boolean autoReturnActive = false;
        boolean replayActive = false;
        boolean recordingActive = false;
        boolean prevA = false;
        boolean prevX = false;
        boolean prevY = false;
        int stableLoopCount = 0;

        List<RecordedCommand> recordedCommands = new ArrayList<>();
        List<RecordedCommand> returnCommands = new ArrayList<>();
        ElapsedTime recordTimer = new ElapsedTime();
        ElapsedTime replayTimer = new ElapsedTime();
        int replayIndex = 0;

        telemetry.addLine("Ready. Press PLAY.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            boolean aNow = gamepad1.a;
            if (aNow && !prevA) {
                autoReturnActive = true;
                replayActive = false;
                stableLoopCount = 0;
            }
            prevA = aNow;

            boolean xNow = gamepad1.x;
            if (xNow && !prevX) {
                if (!recordingActive) {
                    recordingActive = true;
                    replayActive = false;
                    autoReturnActive = false;
                    recordedCommands.clear();
                    recordTimer.reset();
                } else {
                    recordingActive = false;
                }
            }
            prevX = xNow;

            boolean yNow = gamepad1.y;
            if (yNow && !prevY && !recordingActive && !recordedCommands.isEmpty()) {
                replayActive = true;
                autoReturnActive = false;
                returnCommands = createReturnCommands(recordedCommands);
                replayIndex = 0;
                replayTimer.reset();
            }
            prevY = yNow;

            double forwardCmd;
            double turnCmd;

            if (replayActive) {
                if (replayIndex >= returnCommands.size()) {
                    replayActive = false;
                    forwardCmd = 0.0;
                    turnCmd = 0.0;
                } else {
                    while (replayIndex + 1 < returnCommands.size()
                            && replayTimer.seconds() >= returnCommands.get(replayIndex + 1).timeSec) {
                        replayIndex++;
                    }

                    RecordedCommand command = returnCommands.get(replayIndex);
                    forwardCmd = command.forward;
                    turnCmd = command.turn;
                }
            } else if (autoReturnActive) {
                double errorXField = TARGET_X - pose.position.x;
                double errorYField = TARGET_Y - pose.position.y;
                double distance = Math.hypot(errorXField, errorYField);
                double heading = pose.heading.toDouble();

                if (distance <= POSITION_TOLERANCE_INCH) {
                    stableLoopCount++;
                } else {
                    stableLoopCount = 0;
                }

                if (stableLoopCount >= STABLE_LOOP_COUNT_REQUIRED) {
                    autoReturnActive = false;
                    forwardCmd = 0.0;
                    turnCmd = 0.0;
                } else {
                    double targetHeading = Math.atan2(errorYField, errorXField);
                    double headingError = normalizeAngle(targetHeading - heading);

                    turnCmd = Range.clip(
                            HEADING_KP * headingError,
                            -MAX_AUTO_TURN,
                            MAX_AUTO_TURN
                    );

                    if (Math.abs(headingError) <= DRIVE_HEADING_WINDOW_RAD) {
                        forwardCmd = Range.clip(
                                POSITION_KP * distance,
                                -MAX_AUTO_FORWARD,
                                MAX_AUTO_FORWARD
                        );

                        if (distance > POSITION_TOLERANCE_INCH) {
                            forwardCmd = enforceMinMagnitude(forwardCmd, MIN_AUTO_FORWARD);
                        }
                    } else {
                        forwardCmd = 0.0;
                    }
                }
            } else {
                forwardCmd = -applyDeadzone(gamepad1.left_stick_y) * MANUAL_SPEED_SCALE;
                turnCmd = -applyDeadzone(gamepad1.right_stick_x) * MANUAL_SPEED_SCALE;

                if (recordingActive && recordedCommands.size() < MAX_RECORDED_SAMPLES) {
                    recordedCommands.add(new RecordedCommand(
                            recordTimer.seconds(),
                            forwardCmd,
                            turnCmd
                    ));
                }
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardCmd, 0.0), turnCmd));

            String mode;
            if (replayActive) {
                mode = "RETURN_BY_RECORD";
            } else if (autoReturnActive) {
                mode = "AUTO_RETURN";
            } else if (recordingActive) {
                mode = "TELEOP_RECORDING";
            } else {
                mode = "TELEOP";
            }

            telemetry.addData("Mode", mode);
            telemetry.addData("X (in)", "%.2f", pose.position.x);
            telemetry.addData("Y (in)", "%.2f", pose.position.y);
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Auto Stable Loops", stableLoopCount);
            telemetry.addData("Recorded Samples", recordedCommands.size());
            telemetry.addData("Replay Index", replayIndex);
            telemetry.addLine("Controls: A=ReturnTo0, X=Start/Stop Record, Y=ReturnByRecord");
            telemetry.update();
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private static double enforceMinMagnitude(double value, double minMagnitude) {
        if (Math.abs(value) < 1e-6) return 0.0;
        if (Math.abs(value) >= minMagnitude) return value;
        return Math.copySign(minMagnitude, value);
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }

    private static List<RecordedCommand> createReturnCommands(List<RecordedCommand> recorded) {
        List<RecordedCommand> result = new ArrayList<>();
        if (recorded.isEmpty()) {
            return result;
        }

        double endTime = recorded.get(recorded.size() - 1).timeSec;
        for (int i = recorded.size() - 1; i >= 0; i--) {
            RecordedCommand cmd = recorded.get(i);
            double replayTime = endTime - cmd.timeSec;
            result.add(new RecordedCommand(
                    replayTime,
                    -cmd.forward,
                    -cmd.turn
            ));
        }

        return result;
    }

    private static class RecordedCommand {
        final double timeSec;
        final double forward;
        final double turn;

        RecordedCommand(double timeSec, double forward, double turn) {
            this.timeSec = timeSec;
            this.forward = forward;
            this.turn = turn;
        }
    }
}
