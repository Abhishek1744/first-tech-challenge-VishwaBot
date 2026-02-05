package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class RobotHardwareEssentials {

    // Drive motors
    public DcMotor motorLeftFront;
    public DcMotor motorLeftBack;
    public DcMotor motorRightFront;
    public DcMotor motorRightBack;

    // Pinpoint (I2C) odometry
    public GoBildaPinpointDriver pinpoint;

    // ---------------- CONSTANTS ----------------

    // Odometry wheel + encoder constants (4-Bar Mini Odometry Pod, 32mm wheel)
    public static final double ODO_WHEEL_DIAMETER_MM = 32.0;
    public static final double ODO_TICKS_PER_REV = 8192.0;
    public static final double ODO_MM_PER_TICK =
            (Math.PI * ODO_WHEEL_DIAMETER_MM) / ODO_TICKS_PER_REV;

    // Pod offsets from robot center (in encoder ticks). Update these after measuring.
    public static final double PAR_Y_TICKS = 0.0;
    public static final double PERP_X_TICKS = 0.0;

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

        // Pinpoint device (I2C)
        // TODO: ensure your config has a Pinpoint device named "pinpoint"
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(1 / ODO_MM_PER_TICK, DistanceUnit.MM);
        pinpoint.setOffsets(ODO_MM_PER_TICK * PAR_Y_TICKS, ODO_MM_PER_TICK * PERP_X_TICKS, DistanceUnit.MM);
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

        motorLeftFront.setPower(lf);
        motorLeftBack.setPower(lb);
        motorRightFront.setPower(rf);
        motorRightBack.setPower(rb);
    }

    public void stop() {
        setDrivePower(0, 0, 0);
    }

    // ---------------- PINPOINT ODOMETRY (X/Y) ----------------

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
