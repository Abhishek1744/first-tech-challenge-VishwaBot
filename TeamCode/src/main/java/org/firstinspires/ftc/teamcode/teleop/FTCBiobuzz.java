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
        // Motor directions - Realigned with project standard (right side reversed)
        // -------------------------

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // -------------------------
        // Brake
        // -------------------------

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        pinpoint.setEncoderResolution(1.0 / ODO_MM_PER_TICK, DistanceUnit.MM);
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

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

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
            handleButtons();

            // Prioritize AprilTag actions, then specialized states, then TeleOp
            boolean tagAction = handleAprilTags();

            if (!tagAction) {
                if (autoReturning) {
                    DriveCommand autoCommand = calculateAutoReturn();
                    applyDriveCommand(autoCommand.forward, autoCommand.strafe, autoCommand.turn);
                } else if (replaying) {
                    playRecordedPath();
                } else {
                    // Normal Driver Control
                    DriveCommand command = getDriverCommand();
                    applyDriveCommand(command.forward, command.strafe, command.turn);

                    if (recording) {
                        recordCommand(command);
                    }
                }
            }

            updateTelemetry();
            idle();
        }

        stopRobot();
    }

    // ============================================================
    // DRIVER CONTROLS
    // ============================================================

    private DriveCommand getDriverCommand() {

        double forward = -gamepad1.left_stick_y * DRIVE_SPEED_SCALE;
        double strafe = -gamepad1.right_stick_y * DRIVE_SPEED_SCALE;
        double turn = gamepad1.right_stick_x * TURN_SPEED_SCALE;

        forward = applyDeadzone(forward);
        strafe = applyDeadzone(strafe);
        turn = applyDeadzone(turn);

        if (FIELD_ORIENTED) {
            double heading = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            double robotForward = forward * cos + strafe * sin;
            double robotStrafe = -forward * sin + strafe * cos;

            forward = robotForward;
            strafe = robotStrafe;
        }

        return new DriveCommand(forward, strafe, turn);
    }

    // ============================================================
    // DRIVE
    // ============================================================

    private void applyDriveCommand(double forward, double strafe, double turn) {

        double lf = forward + strafe + turn;
        double lb = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rb = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));

        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightFront.setPower(rf / max);
        rightBack.setPower(rb / max);
    }

    // ============================================================
    // BUTTONS
    // ============================================================

    private void handleButtons() {
        // A = Auto Return
        if (gamepad1.a && !lastAutoButton) {
            autoReturning = !autoReturning;
            if (autoReturning) {
                stablePositionLoops = 0;
                replaying = false;
                recording = false;
            }
        }
        lastAutoButton = gamepad1.a;

        // B = Servo
        if (gamepad1.b && !lastServoButton) {
            servosRunning = !servosRunning;
            setServoPairPower(servo1, servo2, servosRunning ? CLOSE_SERVO_POWER : 0.0);
        }
        lastServoButton = gamepad1.b;

        // X = Recording
        if (gamepad1.x && !lastRecordButton) {
            recording = !recording;
            if (recording) {
                recordedPath.clear();
                replaying = false;
                autoReturning = false;
            }
        }
        lastRecordButton = gamepad1.x;

        // Y = Reverse Replay
        if (gamepad1.y && !lastReplayButton) {
            if (!recordedPath.isEmpty()) {
                replaying = !replaying;
                recording = false;
                autoReturning = false;
                if (replaying) {
                    replayIndex = recordedPath.size() - 1;
                }
            }
        }
        lastReplayButton = gamepad1.y;
    }

    // ============================================================
    // APRILTAG
    // ============================================================

    private boolean handleAprilTags() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        boolean tag4 = false, tag5 = false;
        for (HuskyLens.Block b : blocks) {
            if (b.id == TAG_FORWARD_ID) tag4 = true;
            if (b.id == TAG_TURN_ID) tag5 = true;
        }

        if (tag5) {
            if (!tag5Turning) { tag5Turning = true; tagActionTimer.reset(); }
            if (tagActionTimer.seconds() < TAG_TURN_TIME_SECONDS) applyDriveCommand(0, 0, TAG_TURN_POWER);
            else applyDriveCommand(0, 0, 0);
            return true;
        }
        tag5Turning = false;

        if (tag4) {
            if (!tag4Moving) {
                tag4Moving = true;
                tag4StartX = pinpoint.getPosX(DistanceUnit.CM);
                tag4StartY = pinpoint.getPosY(DistanceUnit.CM);
                tagActionTimer.reset();
            }
            double dist = Math.hypot(pinpoint.getPosX(DistanceUnit.CM) - tag4StartX, pinpoint.getPosY(DistanceUnit.CM) - tag4StartY);
            if (dist < TAG_MOVE_DISTANCE_CM && tagActionTimer.seconds() < TAG_MOVE_TIMEOUT_SECONDS) applyDriveCommand(TAG_MOVE_POWER, 0, 0);
            else applyDriveCommand(0, 0, 0);
            return true;
        }
        tag4Moving = false;
        return false;
    }

    // ============================================================
    // AUTO RETURN
    // ============================================================

    private DriveCommand calculateAutoReturn() {
        double curX = pinpoint.getPosX(DistanceUnit.CM);
        double curY = pinpoint.getPosY(DistanceUnit.CM);
        double head = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);

        double errX = TARGET_X_CM - curX;
        double errY = TARGET_Y_CM - curY;
        double dist = Math.hypot(errX, errY);

        if (dist <= POSITION_TOLERANCE_CM) {
            if (++stablePositionLoops >= STABLE_LOOPS_REQUIRED) autoReturning = false;
            return new DriveCommand(0, 0, 0);
        }
        stablePositionLoops = 0;

        double cos = Math.cos(head);
        double sin = Math.sin(head);
        double robotFwd = errX * cos + errY * sin;
        double robotStr = -errX * sin + errY * cos;

        double max = (dist <= NEAR_TARGET_RADIUS_CM) ? MAX_AUTO_TRANSLATION_NEAR : MAX_AUTO_TRANSLATION_FAR;
        double fwd = Range.clip(robotFwd * POSITION_KP, -max, max);
        double str = Range.clip(robotStr * POSITION_KP, -max, max);

        return new DriveCommand(fwd, str, 0);
    }

    // ============================================================
    // RECORDING / REPLAY
    // ============================================================

    private void recordCommand(DriveCommand command) {
        if (recordedPath.size() < MAX_RECORDING_SAMPLES)
            recordedPath.add(new DriveCommand(command.forward, command.strafe, command.turn));
        else recording = false;
    }

    private void playRecordedPath() {
        if (recordedPath.isEmpty() || replayIndex < 0) {
            replaying = false;
            applyDriveCommand(0, 0, 0);
            return;
        }
        DriveCommand cmd = recordedPath.get(replayIndex--);
        applyDriveCommand(-cmd.forward, -cmd.strafe, -cmd.turn);
    }

    private void setServoPairPower(CRServo s1, CRServo s2, double p) {
        s1.setPower(p);
        s2.setPower(-p);
    }

    private double applyDeadzone(double v) {
        return (Math.abs(v) < DRIVE_DEADZONE) ? 0.0 : v;
    }

    private void updateTelemetry() {
        telemetry.addData("Status", autoReturning ? "Auto Returning" : (replaying ? "Replaying" : (recording ? "Recording" : "TeleOp")));
        telemetry.addData("Pos (CM)", "X:%.1f Y:%.1f H:%.1f", pinpoint.getPosX(DistanceUnit.CM), pinpoint.getPosY(DistanceUnit.CM), Math.toDegrees(pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)));
        telemetry.addData("Recording", "%b (%d samples)", recording, recordedPath.size());
        telemetry.update();
    }

    private void stopRobot() {
        applyDriveCommand(0, 0, 0);
        setServoPairPower(servo1, servo2, 0);
    }
}
