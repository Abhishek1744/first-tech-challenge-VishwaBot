package GamePadAllCodesEXAMPLE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "VishwaBot Drive + Rotate", group = "Drive")
public class gamePadBumper extends OpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;

    double speedMultiplier = 1.0;   // Default speed = 100%

    @Override
    public void init() {

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");


        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("VishwaBot Ready");
        telemetry.update();
    }

    @Override
    public void loop() {


        if (gamepad1.right_bumper) {
            speedMultiplier += 0.02;
        }
        if (gamepad1.left_bumper) {
            speedMultiplier -= 0.02;
        }


        speedMultiplier = Math.max(0.2, Math.min(speedMultiplier, 1.0));


        double drive  = -gamepad1.left_stick_y * speedMultiplier;
        double strafe =  gamepad1.left_stick_x * speedMultiplier;
        double turn   =  gamepad1.right_stick_x * speedMultiplier; // 360° rotation


        if (Math.abs(drive) < 0.05) drive = 0;
        if (Math.abs(strafe) < 0.05) strafe = 0;
        if (Math.abs(turn) < 0.05) turn = 0;


        double lf = drive + strafe + turn;
        double rf = drive - strafe - turn;
        double lb = drive - strafe + turn;
        double rb = drive + strafe - turn;


        double max = Math.max(
                Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb))
        );

        if (max > 0.5) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);

        telemetry.addData("Speed", "%.2f", speedMultiplier);
        telemetry.addData("Turn", "%.2f", turn);
        telemetry.update();
    }
}
