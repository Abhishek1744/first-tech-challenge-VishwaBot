package mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TestBenchIMU;
@Autonomous
public class stateIntegerPractice extends OpMode {

    TestBench bench = new TestBench();

    int state;

    @Override
    public void init() {
    bench.init(hardwareMap);
    state = 0;
    }

    @Override
    public void loop() {
    telemetry.addData("Cur State", state);
    switch (state) {
        case 0:
            telemetry.addLine("To exit state, press A");
            if (gamepad1.a) {
                state = 1;
            }
            break;
        case 1:
            telemetry.addLine("To exit state, press B");
            if (gamepad1.b) {
                state = 2;
            }
            break;
        case 2:
            telemetry.addLine("To exit state, press X");
            if (gamepad1.x) {
                state = 3;
            }
            break;
        default:
            telemetry.addLine("Auto State machine finished");
    }
    }
}
