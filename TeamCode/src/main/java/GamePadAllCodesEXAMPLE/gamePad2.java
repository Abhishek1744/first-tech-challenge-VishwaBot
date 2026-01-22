package GamePadAllCodesEXAMPLE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class gamePad2 extends OpMode {
    boolean intakeOn = false;
    boolean lastAState = false;
    boolean shooterOn = false;
    boolean lastXState = false;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        boolean currentAState = gamepad2.a;
        boolean currentXState = gamepad2.x;

        if (currentAState && !lastAState){
            intakeOn = !intakeOn;
        }
        if (currentXState && !lastXState){
            shooterOn = !shooterOn;
        }
        lastAState = currentAState;
        lastXState = currentXState;

    }
}
