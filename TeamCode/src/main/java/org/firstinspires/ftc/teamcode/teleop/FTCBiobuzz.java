package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "FTC Field Oriented", group = "TeleOp")
public class FTCBiobuzz extends LinearOpMode {

    // =========================
    // GENERAL SETTINGS
    // =========================

    private static final double CLOSE_SERVO_POWER = 1.0;

    private static final double DRIVE_DEADZONE = 0.12;
    private static final double DRIVE_SPEED_SCALE = 1.0;
    private static final double TURN_SPEED_SCALE = 1.0;

    // Field-oriented drive
    private static final boolean FIELD_ORIENTED = true;

    // =========================
    // PINPOINT SETTINGS
    // =========================

    private static final double ODO_WHEEL_DIAMETER_MM = 32.0;
    private static final double ODO_TICKS_PER_REV = 8192.0;

    private static final double ODO_MM_PER_TICK =
            (Math.PI * ODO_WHEEL_DIAMETER_MM) / ODO_TICKS_PER_REV;

    // =========================
    // AUTO RETURN SETTINGS
    // =========================

    private static final double POSITION_KP = 0.04;

    private static final double MAX_AUTO_TRANSLATION_FAR = 0.20;
    private static final double MAX_AUTO_TRANSLATION_NEAR = 0.10;

    private static final double MIN_AUTO_TRANSLATION = 0.04;

    private static final double NEAR_TARGET_RADIUS_CM = 15.24;
    private static final double POSITION_TOLERANCE_CM = 5.08;

    private static final int STABLE_LOOPS_REQUIRED = 3;

    private static final double TARGET_X_CM = 0.0;
    private static final double TARGET_Y_CM = 0.0;

    // =========================
    // RECORDING SETTINGS
    // =========================

    private static final int MAX_RECORDING_SAMPLES = 6000;

    // =========================
    // HUSKYLENS / APRILTAG
    // =========================

    private static final String HUSKYLENS_NAME = "huskyLens";

    private static final int TAG_FORWARD_ID = 4;
    private static final int TAG_TURN_ID = 5;

    private static final double TAG_MOVE_DISTANCE_CM = 2.0;
    private static final double TAG_MOVE_POWER = 0.12;
    private static final double TAG_MOVE_TIMEOUT_SECONDS = 2.0;

    private static final double TAG_TURN_POWER = 0.20;
    private static final double TAG_TURN_TIME_SECONDS = 0.80;

    // =========================
    // HARDWARE
    // =========================

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private CRServo servo1;
    private CRServo servo2;

    private GoBildaPinpointDriver pinpoint;
    private HuskyLens huskyLens;

    // =========================
    // RECORDING DATA
    // =========================

    private static class DriveCommand {
        double forward;
        double strafe;
        double turn;

        DriveCommand(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
    }

    private final List<DriveCommand> recordedPath = new ArrayList<>();

    // =========================
    // STATES
    // =========================

    private boolean servosRunning = false;
    private boolean recording = false;
    private boolean replaying = false;
    private boolean autoReturning = false;

    private boolean lastServoButton = false;
    private boolean lastRecordButton = false;
    private boolean lastReplayButton = false;
    private boolean lastAutoButton = false;

    private int replayIndex = 0;

    private int stablePositionLoops = 0;

    private boolean tag4Moving = false;
    private boolean tag5Turning = false;

    private double tag4StartX = 0.0;
    private double tag4StartY = 0.0;

    private final ElapsedTime tagActionTimer = new ElapsedTime();

    // =========================
    // INIT
    // =========================

    @Override
    public void runOpMode() {

        // -------------------------
        // Hardware
        // -------------------------

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        servo1 = hardwareMap.get(CRServo.class, "Servo1");
        servo2 = hardwareMap.get(CRServo.class, "Servo2");

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        huskyLens = hardwareMap.get(
                HuskyLens.class,
                HUSKYLENS_NAME
        );

        // -------------------------
        // Motor directions
        // -------------------------

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // -------------------------
        // Brake
        // -------------------------

        leftFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        leftBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // -------------------------
        // Motor modes
        // -------------------------

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // -------------------------
        // Pinpoint
        // -------------------------

        pinpoint.setEncoderResolution(
                1.0 / ODO_MM_PER_TICK,
                DistanceUnit.MM
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        // -------------------------
        // Servo
        // -------------------------

        setServoPairPower(servo1, servo2, 0.0);

        // -------------------------
        // HuskyLens
        // -------------------------

        huskyLens.selectAlgorithm(
                HuskyLens.Algorithm.TAG_RECOGNITION
        );

        telemetry.addLine("Testing 2027 Field Oriented Ready");
        telemetry.addLine("");
        telemetry.addLine("FIELD ORIENTED DRIVE: ON");
        telemetry.addLine("Left Y  = Forward / Backward");
        telemetry.addLine("Right Y = Strafe");
        telemetry.addLine("Right X = Turn");
        telemetry.addLine("");
        telemetry.addLine("A = Auto Return");
        telemetry.addLine("B = Servo");
        telemetry.addLine("X = Record");
        telemetry.addLine("Y = Reverse Replay");
        telemetry.addLine("");
        telemetry.addLine("AprilTag 4 = Forward");
        telemetry.addLine("AprilTag 5 = Turn");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // =========================
        // MAIN LOOP
        // =========================

        while (opModeIsActive()) {

            pinpoint.update();

            // -------------------------
            // Button handling
            // -------------------------

            handleButtons();

            // -------------------------
            // AprilTag
            // -------------------------

            boolean aprilTagAction = handleAprilTags();

            // -------------------------
            // Auto Return
            // -------------------------

            if (!aprilTagAction && autoReturning) {

                DriveCommand autoCommand =
                        calculateAutoReturn();

                applyDriveCommand(
                        autoCommand.forward,
                        autoCommand.strafe,
                        autoCommand.turn
                );
            }

            // -------------------------
            // Replay
            // -------------------------

            else if (!aprilTagAction && replaying) {

                playRecordedPath();
            }

            // -------------------------
            // Recording
            // -------------------------

            else if (!aprilTagAction && recording) {

                DriveCommand command =
                        getDriverCommand();

                applyDriveCommand(
                        command.forward,
                        command.strafe,
                        command.turn
                );

                recordCommand(command);
            }

            // -------------------------
            // Normal TeleOp
            // -------------------------

            else if (!aprilTagAction && !autoReturning && !replaying) {

                DriveCommand command =
                        getDriverCommand();

                applyDriveCommand(
                        command.forward,
                        command.strafe,
                        command.turn
                );
            }

            // -------------------------
            // Telemetry
            // -------------------------

            updateTelemetry();

            idle();
        }

        // =========================
        // STOP
        // =========================

        stopRobot();
    }

    // ============================================================
    // DRIVER CONTROLS
    // ============================================================

    private DriveCommand getDriverCommand() {

        // -------------------------
        // Raw joystick input
        // -------------------------

        double forward =
                -gamepad1.left_stick_y
                        * DRIVE_SPEED_SCALE;

        double strafe =
                -gamepad1.right_stick_y
                        * DRIVE_SPEED_SCALE;

        double turn =
                gamepad1.right_stick_x
                        * TURN_SPEED_SCALE;

        forward = applyDeadzone(forward);
        strafe = applyDeadzone(strafe);
        turn = applyDeadzone(turn);

        // ========================================================
        // FIELD ORIENTED TRANSFORMATION
        // ========================================================

        if (FIELD_ORIENTED) {

            double heading =
                    pinpoint.getHeading(
                            UnnormalizedAngleUnit.RADIANS
                    );

            double cosHeading = Math.cos(heading);
            double sinHeading = Math.sin(heading);

            double robotForward =
                    forward * cosHeading
                            + strafe * sinHeading;

            double robotStrafe =
                    -forward * sinHeading
                            + strafe * cosHeading;

            forward = robotForward;
            strafe = robotStrafe;
        }

        return new DriveCommand(
                forward,
                strafe,
                turn
        );
    }

    // ============================================================
    // DRIVE
    // ============================================================

    private void applyDriveCommand(
            double forward,
            double strafe,
            double turn
    ) {

        double leftFrontPower =
                forward + strafe + turn;

        double leftBackPower =
                forward - strafe + turn;

        double rightFrontPower =
                forward - strafe - turn;

        double rightBackPower =
                forward + strafe - turn;

        double denominator = Math.max(
                1.0,
                Math.max(
                        Math.abs(leftFrontPower),
                        Math.max(
                                Math.abs(leftBackPower),
                                Math.max(
                                        Math.abs(rightFrontPower),
                                        Math.abs(rightBackPower)
                                )
                        )
                )
        );

        leftFrontPower /= denominator;
        leftBackPower /= denominator;
        rightFrontPower /= denominator;
        rightBackPower /= denominator;

        leftFront.setPower(
                Range.clip(leftFrontPower, -1.0, 1.0)
        );

        leftBack.setPower(
                Range.clip(leftBackPower, -1.0, 1.0)
        );

        rightFront.setPower(
                Range.clip(rightFrontPower, -1.0, 1.0)
        );

        rightBack.setPower(
                Range.clip(rightBackPower, -1.0, 1.0)
        );
    }

    // ============================================================
    // BUTTONS
    // ============================================================

    private void handleButtons() {

        // -------------------------
        // A = Auto Return
        // -------------------------

        if (gamepad1.a && !lastAutoButton) {

            autoReturning = !autoReturning;

            if (autoReturning) {
                stablePositionLoops = 0;
                replaying = false;
            }
        }

        lastAutoButton = gamepad1.a;

        // -------------------------
        // B = Servo
        // -------------------------

        if (gamepad1.b && !lastServoButton) {

            servosRunning = !servosRunning;

            setServoPairPower(
                    servo1,
                    servo2,
                    servosRunning
                            ? CLOSE_SERVO_POWER
                            : 0.0
            );
        }

        lastServoButton = gamepad1.b;

        // -------------------------
        // X = Recording
        // -------------------------

        if (gamepad1.x && !lastRecordButton) {

            recording = !recording;

            if (recording) {

                recordedPath.clear();
                replaying = false;
                autoReturning = false;
            }
        }

        lastRecordButton = gamepad1.x;

        // -------------------------
        // Y = Reverse Replay
        // -------------------------

        if (gamepad1.y && !lastReplayButton) {

            if (!recordedPath.isEmpty()) {

                replaying = !replaying;
                recording = false;
                autoReturning = false;

                if (replaying) {
                    replayIndex =
                            recordedPath.size() - 1;
                }
            }
        }

        lastReplayButton = gamepad1.y;
    }

    // ============================================================
    // APRILTAG
    // ============================================================

    private boolean handleAprilTags() {

        HuskyLens.Block[] blocks =
                huskyLens.blocks();

        boolean tag4Visible = false;
        boolean tag5Visible = false;

        for (HuskyLens.Block block : blocks) {

            if (block.id == TAG_FORWARD_ID) {
                tag4Visible = true;
            }

            if (block.id == TAG_TURN_ID) {
                tag5Visible = true;
            }
        }

        // -------------------------
        // TAG 5 = TURN
        // -------------------------

        if (tag5Visible) {

            if (!tag5Turning) {

                tag5Turning = true;
                tagActionTimer.reset();
            }

            double elapsed =
                    tagActionTimer.seconds();

            if (elapsed < TAG_TURN_TIME_SECONDS) {

                applyDriveCommand(
                        0.0,
                        0.0,
                        TAG_TURN_POWER
                );

            } else {

                applyDriveCommand(
                        0.0,
                        0.0,
                        0.0
                );
            }

            return true;
        }

        tag5Turning = false;

        // -------------------------
        // TAG 4 = FORWARD
        // -------------------------

        if (tag4Visible) {

            if (!tag4Moving) {

                tag4Moving = true;

                tag4StartX =
                        pinpoint.getPosX(DistanceUnit.CM);

                tag4StartY =
                        pinpoint.getPosY(DistanceUnit.CM);

                tagActionTimer.reset();
            }

            double currentX =
                    pinpoint.getPosX(DistanceUnit.CM);

            double currentY =
                    pinpoint.getPosY(DistanceUnit.CM);

            double dx =
                    currentX - tag4StartX;

            double dy =
                    currentY - tag4StartY;

            double distance =
                    Math.sqrt(
                            dx * dx +
                                    dy * dy
                    );

            double elapsed =
                    tagActionTimer.seconds();

            if (
                    distance < TAG_MOVE_DISTANCE_CM
                            &&
                            elapsed < TAG_MOVE_TIMEOUT_SECONDS
            ) {

                applyDriveCommand(
                        TAG_MOVE_POWER,
                        0.0,
                        0.0
                );

            } else {

                applyDriveCommand(
                        0.0,
                        0.0,
                        0.0
                );
            }

            return true;
        }

        tag4Moving = false;

        return false;
    }

    // ============================================================
    // AUTO RETURN
    // ============================================================

    private DriveCommand calculateAutoReturn() {

        double currentX =
                pinpoint.getPosX(DistanceUnit.CM);

        double currentY =
                pinpoint.getPosY(DistanceUnit.CM);

        double heading =
                pinpoint.getHeading(
                        UnnormalizedAngleUnit.RADIANS
                );

        double errorX =
                TARGET_X_CM - currentX;

        double errorY =
                TARGET_Y_CM - currentY;

        double distance =
                Math.sqrt(
                        errorX * errorX +
                                errorY * errorY
                );

        if (distance <= POSITION_TOLERANCE_CM) {

            stablePositionLoops++;

            if (
                    stablePositionLoops
                            >= STABLE_LOOPS_REQUIRED
            ) {

                autoReturning = false;

                return new DriveCommand(
                        0.0,
                        0.0,
                        0.0
                );
            }

            return new DriveCommand(
                    0.0,
                    0.0,
                    0.0
            );
        }

        stablePositionLoops = 0;

        // Field-to-robot transformation

        double cos =
                Math.cos(heading);

        double sin =
                Math.sin(heading);

        double robotForward =
                errorY * cos +
                        errorX * sin;

        double robotStrafe =
                -errorY * sin +
                        errorX * cos;

        double forward =
                robotForward * POSITION_KP;

        double strafe =
                robotStrafe * POSITION_KP;

        double maxTranslation;

        if (
                distance
                        <= NEAR_TARGET_RADIUS_CM
        ) {
            maxTranslation =
                    MAX_AUTO_TRANSLATION_NEAR;
        } else {
            maxTranslation =
                    MAX_AUTO_TRANSLATION_FAR;
        }

        forward = Range.clip(
                forward,
                -maxTranslation,
                maxTranslation
        );

        strafe = Range.clip(
                strafe,
                -maxTranslation,
                maxTranslation
        );

        double magnitude =
                Math.sqrt(
                        forward * forward +
                                strafe * strafe
                );

        if (
                magnitude > 0.0
                        &&
                        magnitude < MIN_AUTO_TRANSLATION
        ) {

            double scale =
                    MIN_AUTO_TRANSLATION /
                            magnitude;

            forward *= scale;
            strafe *= scale;

            magnitude =
                    Math.sqrt(
                            forward * forward +
                                    strafe * strafe
                    );

            if (magnitude > maxTranslation) {

                double maxScale =
                        maxTranslation /
                                magnitude;

                forward *= maxScale;
                strafe *= maxScale;
            }
        }

        double turn = 0.0;

        return new DriveCommand(
                forward,
                strafe,
                turn
        );
    }

    // ============================================================
    // RECORDING
    // ============================================================

    private void recordCommand(
            DriveCommand command
    ) {

        if (
                recordedPath.size()
                        >= MAX_RECORDING_SAMPLES
        ) {
            recording = false;
            return;
        }

        recordedPath.add(
                new DriveCommand(
                        command.forward,
                        command.strafe,
                        command.turn
                )
        );
    }

    // ============================================================
    // REPLAY
    // ============================================================

    private void playRecordedPath() {

        if (
                recordedPath.isEmpty()
                        ||
                        replayIndex < 0
                        ||
                        replayIndex >= recordedPath.size()
        ) {

            replaying = false;

            applyDriveCommand(
                    0.0,
                    0.0,
                    0.0
            );

            return;
        }

        DriveCommand original =
                recordedPath.get(replayIndex);

        double forward =
                -original.forward;

        double strafe =
                -original.strafe;

        double turn =
                -original.turn;

        applyDriveCommand(
                forward,
                strafe,
                turn
        );

        replayIndex--;

        if (replayIndex < 0) {

            replaying = false;

            applyDriveCommand(
                    0.0,
                    0.0,
                    0.0
            );
        }
    }

    // ============================================================
    // SERVO
    // ============================================================

    private void setServoPairPower(
            CRServo servo1,
            CRServo servo2,
            double power
    ) {

        servo1.setPower(power);
        servo2.setPower(-power);
    }

    // ============================================================
    // DEADZONE
    // ============================================================

    private double applyDeadzone(
            double value
    ) {

        if (
                Math.abs(value)
                        < DRIVE_DEADZONE
        ) {
            return 0.0;
        }

        return value;
    }

    // ============================================================
    // TELEMETRY
    // ============================================================

    private void updateTelemetry() {

        telemetry.addLine(
                "===== TESTING 2027 ====="
        );

        telemetry.addLine(
                "FIELD ORIENTED: ON"
        );

        telemetry.addLine(
                "Left Y  = Forward / Back"
        );

        telemetry.addLine(
                "Right Y = Strafe"
        );

        telemetry.addLine(
                "Right X = Turn"
        );

        telemetry.addLine("");

        telemetry.addData(
                "Servo",
                servosRunning
                        ? "RUNNING"
                        : "STOPPED"
        );

        telemetry.addData(
                "Recording",
                recording
        );

        telemetry.addData(
                "Recorded Samples",
                recordedPath.size()
        );

        telemetry.addData(
                "Replay",
                replaying
        );

        telemetry.addData(
                "Auto Return",
                autoReturning
        );

        // -------------------------
        // Pinpoint
        // -------------------------

        if (
                pinpoint.getDeviceStatus()
                        ==
                        GoBildaPinpointDriver.DeviceStatus.READY
        ) {

            telemetry.addData(
                    "X (cm)",
                    "%.2f",
                    pinpoint.getPosX(
                            DistanceUnit.CM
                    )
            );

            telemetry.addData(
                    "Y (cm)",
                    "%.2f",
                    pinpoint.getPosY(
                            DistanceUnit.CM
                    )
            );

            telemetry.addData(
                    "Heading (deg)",
                    "%.1f",
                    Math.toDegrees(
                            pinpoint.getHeading(
                                    UnnormalizedAngleUnit.RADIANS
                            )
                    )
            );

        } else {

            telemetry.addData(
                    "Odometry",
                    pinpoint.getDeviceStatus()
            );
        }

        // -------------------------
        // Motor powers
        // -------------------------

        telemetry.addData(
                "LF",
                "%.2f",
                leftFront.getPower()
        );

        telemetry.addData(
                "LB",
                "%.2f",
                leftBack.getPower()
        );

        telemetry.addData(
                "RF",
                "%.2f",
                rightFront.getPower()
        );

        telemetry.addData(
                "RB",
                "%.2f",
                rightBack.getPower()
        );

        telemetry.addLine("");

        telemetry.addLine(
                "AprilTag 4 = FORWARD"
        );

        telemetry.addLine(
                "AprilTag 5 = TURN"
        );

        telemetry.addData(
                "Tag 4 Moving",
                tag4Moving
        );

        telemetry.addData(
                "Tag 5 Turning",
                tag5Turning
        );

        telemetry.update();
    }

    // ============================================================
    // STOP
    // ============================================================

    private void stopRobot() {

        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);

        setServoPairPower(
                servo1,
                servo2,
                0.0
        );
    }
}