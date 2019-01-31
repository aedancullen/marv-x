package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MoveHinge")
public class MarvXIMoveHinge extends OpMode {

    MarvXICommon marv;

    boolean hingeInHoldMode = false;

    public void init() {
        marv = new MarvXICommon(hardwareMap);
    }

    public void loop() {
        double g2HingeInput = gamepad2.left_stick_y * 0.2;
        if (gamepad2.dpad_up) {
            g2HingeInput -= 0.5;
        }
        else if (gamepad2.dpad_down) {
            g2HingeInput += 0.5;
        }
        if (g2HingeInput != 0) {
            if (hingeInHoldMode) {
                marv.setHingeDefaultMode();
                hingeInHoldMode = false;
            }
            if (false/*g2HingeInput < 0 && (marv.hingeL.getCurrentPosition() + marv.hingeR.getCurrentPosition()) / 2.0 <= 0*/) {marv.setHingeSpeed(0);} else {marv.setHingeSpeed(g2HingeInput);}
        }
        else {
            /*if (!hingeInHoldMode) {
                marv.setHingeHoldMode();
                hingeInHoldMode = true;
            }*/
            marv.setHingeSpeed(0);
        }
    }

    public void stop() {
        marv.stop();
    }
}
