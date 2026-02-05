package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class RobotHardwareEssentials {

    // Drive motors
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    // Pinpoint (I2C) odometry
    public GoBildaPinpointDriver pinpoint;

    // ---------------- CONSTANTS ----------------

    // 32mm odometry wheel, 8192 CPR
    public static final double ODO_WHEEL_DIAMETER_MM = 32.0;
    public static final double ODO_TICKS_PER_REV = 8192.0;
    public static final double ODO_MM_PER_TICK =
            (Math.PI * ODO_WHEEL_DIAMETER_MM) / ODO_TICKS_PER_REV;

    // Update after physical measurement
    public static final double PAR_Y_TICKS = 0.0;
    public static final double PERP_X_TICKS = 0.0;

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

        // Pinpoint device
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(1 / ODO_MM_PER_TICK, DistanceUnit.MM);
        pinpoint.setOffsets(
                ODO_MM_PER_TICK * PAR_Y_TICKS,
                ODO_MM_PER_TICK * PERP_X_TICKS,
                DistanceUnit.MM
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();
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

    // ---------------- PINPOINT ODOMETRY ----------------

    public void updateOdometry() {
        pinpoint.update();

        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            return;
        }

        pos.x = pinpoint.getPosX(DistanceUnit.CM);
        pos.y = pinpoint.getPosY(DistanceUnit.CM);
        pos.h = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);
    }
}
