package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardwareEssentials {

    // Drive motors
    public DcMotor motorLeftFront;
    public DcMotor motorLeftBack;
    public DcMotor motorRightFront;
    public DcMotor motorRightBack;

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

        motorLeftFront  = hwMap.get(DcMotor.class, "motorLeftFront");
        motorLeftBack   = hwMap.get(DcMotor.class, "motorLeftBack");
        motorRightFront = hwMap.get(DcMotor.class, "motorRightFront");
        motorRightBack  = hwMap.get(DcMotor.class, "motorRightBack");

        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);

        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Assign encoders
        encoderLeft = motorLeftBack;
        encoderRight = motorRightBack;

        resetEncoders();
    }

    // ---------------- DRIVE ----------------

    public void setDrivePower(double drive, double strafe, double turn) {

        double lf = drive - strafe - turn;
        double lb = drive + strafe - turn;
        double rf = drive + strafe + turn;
        double rb = drive - strafe + turn;

        motorLeftFront.setPower(lf);
        motorLeftBack.setPower(lb);
        motorRightFront.setPower(rf);
        motorRightBack.setPower(rb);
    }

    public void stop() {
        setDrivePower(0, 0, 0);
    }

    // ---------------- ENCODERS ----------------

    public void resetEncoders() {
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
