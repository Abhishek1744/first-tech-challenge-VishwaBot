package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardwareEssentials {

    // Drive motors
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    // Encoders (2 only)
    public DcMotor encoderLeft;
    public DcMotor encoderRight;

    // ---------------- CONSTANTS ----------------

    // Distance between left and right wheels (cm)
    public static final double TRACK_WIDTH_CM = 38.0;

    // Wheel radius (cm)
    public static final double WHEEL_RADIUS_CM = 5.2;

    // Encoder ticks per revolution
    public static final double TICKS_PER_REV = 8192.0;

    // cm per encoder tick
    public static final double CM_PER_TICK =
            (2 * Math.PI * WHEEL_RADIUS_CM) / TICKS_PER_REV;

    // ---------------- ODOMETRY STATE ----------------

    private int lastLeftPos = 0;
    private int lastRightPos = 0;

    public XyhVector pos = new XyhVector(0, 0, 0);

    // ---------------- CONSTRUCTOR ----------------

    public RobotHardwareEssentials(HardwareMap hwMap) {

        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        leftBack   = hwMap.get(DcMotor.class, "leftBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Assign encoders
        encoderLeft = leftBack;
        encoderRight = rightBack;

        resetEncoders();
    }

    // ---------------- DRIVE ----------------

    public void setDrivePower(double drive, double strafe, double turn) {

        double lf = drive - strafe - turn;
        double lb = drive + strafe - turn;
        double rf = drive + strafe + turn;
        double rb = drive - strafe + turn;

        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    public void stop() {
        setDrivePower(0, 0, 0);
    }

    // ---------------- ENCODERS ----------------

    public void resetEncoders() {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastLeftPos = 0;
        lastRightPos = 0;
    }

    // ---------------- 2-ENCODER ODOMETRY ----------------

    public void updateOdometry() {

        int leftPos = encoderLeft.getCurrentPosition();
        int rightPos = encoderRight.getCurrentPosition();

        int dLeft = leftPos - lastLeftPos;
        int dRight = rightPos - lastRightPos;

        lastLeftPos = leftPos;
        lastRightPos = rightPos;

        double leftDist = dLeft * CM_PER_TICK;
        double rightDist = dRight * CM_PER_TICK;

        double dTheta = (rightDist - leftDist) / TRACK_WIDTH_CM;
        double dCenter = (rightDist + leftDist) / 2.0;

        double headingMid = pos.h + dTheta / 2.0;

        pos.x += dCenter * Math.cos(headingMid);
        pos.y += dCenter * Math.sin(headingMid);
        pos.h += dTheta;
    }
}
